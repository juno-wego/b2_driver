"""Use the upstream GUI/solver with explicit model and per-camera output paths."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import tarfile
import threading
import uuid

from b2_camera_calibration.configuration import board_settings, camera_identity, output_directory
from camera_calibration.calibrator import CAMERA_MODEL, ChessboardInfo, MonoCalibrator, Patterns
from camera_calibration.camera_calibrator import OpenCVCalibrationNode
import cv2
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.srv import SetCameraInfo


def atomic_text(path, text):
    temporary = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    try:
        temporary.write_text(text)
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


class NamedMonoCalibrator(MonoCalibrator):
    def __init__(self, boards, camera_name, model, output_root):
        super().__init__(
            boards,
            pattern=Patterns.Chessboard,
            name=camera_name,
            fisheye_flags=(cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_CHECK_COND),
        )
        self.set_cammodel(model)
        self.output_root = Path(output_root)
        self.board_description = [
            {'inner_corners': [board.n_cols, board.n_rows], 'square_size_m': board.dim}
            for board in boards
        ]

    def do_save(self):
        if not self.calibrated:
            raise ValueError('Calibrate collected images before saving')
        session = (
            datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '_' + uuid.uuid4().hex[:8]
        )
        directory = self.output_root / 'datasets' / self.name / session
        directory.mkdir(parents=True)
        archive = directory / 'calibrationdata.tar.gz'
        with tarfile.open(archive, 'w:gz') as output:
            self.do_tarfile_save(output)
        calibration = self.output_root / 'camera_info' / (self.name + '.yaml')
        calibration.parent.mkdir(parents=True, exist_ok=True)
        atomic_text(calibration, self.yaml())
        atomic_text(
            directory / 'session.json',
            json.dumps(
                {
                    'camera_name': self.name,
                    'board': self.board_description,
                    'camera_model': self.as_message().distortion_model,
                    'image_size': list(self.size),
                    'image_samples': len(self.db),
                    'calibration_file': str(calibration),
                    'archive': str(archive),
                },
                indent=2,
            )
            + '\n',
        )
        print(f'Saved {self.name}: {calibration}\nDataset: {archive}', flush=True)


class B2CalibrationNode(OpenCVCalibrationNode):
    def __init__(self, side, size, square, model, output_root):
        camera_name, _ = camera_identity(side)
        self.model_name = model
        self.selected_model = {'fisheye': CAMERA_MODEL.FISHEYE, 'pinhole': CAMERA_MODEL.PINHOLE}[
            model
        ]
        self.output_root = output_root
        super().__init__(
            camera_name + '_calibrator',
            [ChessboardInfo('chessboard', size[0], size[1], square)],
            service_check=False,
            pattern=Patterns.Chessboard,
            camera_name=camera_name,
            checkerboard_flags=0,
            queue_size=1,
        )

    def get_topic_qos(self, topic_name):
        # The raw SDK publisher is BEST_EFFORT even before it is discovered.
        return qos_profile_sensor_data

    def initWindow(self):
        cv2.namedWindow('display', cv2.WINDOW_NORMAL)
        cv2.setWindowTitle('display', self._camera_name + ' | ' + self.model_name)
        cv2.setMouseCallback('display', self.on_mouse)
        # The model is selected by the launch argument before processing frames.
        # Keep the upstream preview scale, without its default pinhole selector.
        cv2.createTrackbar('scale', 'display', 0, 100, self.on_scale)

    def handle_monocular(self, message):
        if self.c is None:
            self.c = NamedMonoCalibrator(
                self._boards, self._camera_name, self.selected_model, self.output_root
            )
        super().handle_monocular(message)

    def handle_stereo(self, message):
        self.get_logger().error(
            'B2 front/back cameras must be calibrated as separate monocular cameras'
        )

    def check_set_camera_info(self, response):
        if response is not None and response.success:
            return True
        detail = response.status_message if response is not None else 'Service request timed out'
        self.get_logger().error('Calibration was not committed: ' + detail)
        return False

    def do_upload(self):
        if self.c is None or not self.c.calibrated or self.c.camera_model != self.selected_model:
            self.get_logger().error(
                'Finish calibration with the selected camera model before COMMIT'
            )
            return False
        response = self.set_camera_info_service.call(
            SetCameraInfo.Request(camera_info=self.c.as_message()), timeout_sec=5.0
        )
        return self.check_set_camera_info(response)

    def spin(self):
        executor = SingleThreadedExecutor(context=self.context)
        executor.add_node(self)

        def execute():
            try:
                executor.spin()
            except ExternalShutdownException:
                pass

        worker = threading.Thread(target=execute, daemon=True)
        worker.start()
        self.image = None
        try:
            while rclpy.ok(context=self.context):
                if not self.queue_display.empty():
                    self.image = self.queue_display.get()
                    cv2.imshow('display', self.image)
                if cv2.waitKey(20) & 0xFF in (27, ord('q')):
                    break
                if cv2.getWindowProperty('display', cv2.WND_PROP_VISIBLE) < 1:
                    break
        finally:
            executor.shutdown(timeout_sec=2.0)
            worker.join(timeout=2.0)
            cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera-side', choices=['front', 'back'], required=True)
    parser.add_argument('--board-size', required=True, help='Measured inner corners, e.g. 8x6')
    parser.add_argument(
        '--square-size-m', required=True, help='Measured square side length in meters'
    )
    parser.add_argument('--camera-model', choices=['fisheye', 'pinhole'], default='fisheye')
    parser.add_argument('--output-root', required=True)
    args = parser.parse_args(remove_ros_args()[1:])
    try:
        size, square = board_settings(args.board_size, args.square_size_m)
        root = output_directory(args.output_root)
    except ValueError as exc:
        parser.error(str(exc))
    rclpy.init()
    node = None
    try:
        node = B2CalibrationNode(args.camera_side, size, square, args.camera_model, root)
        if not node.set_camera_info_service.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('Selected camera set_camera_info service is unavailable')
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
