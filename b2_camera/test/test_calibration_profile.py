"""Keep the imported front-camera measurement and its provenance unchanged."""

import hashlib
import json
from pathlib import Path

import yaml


CALIBRATION = Path(__file__).resolve().parents[1] / "config/calibration"


def test_front_profile_preserves_the_pinned_raw_measurement():
    payload = (CALIBRATION / "b2_front_1920x1080.yaml").read_bytes()
    provenance = json.loads((CALIBRATION / "source.json").read_text())
    assert len(payload) == provenance["profile_size_bytes"] == 594
    assert hashlib.sha256(payload).hexdigest() == provenance["profile_sha256"]
    assert payload.count(b"camera_name: b2_front_camera\n") == 1
    original = payload.replace(b"camera_name: b2_front_camera\n", b"camera_name: b2_camera\n", 1)
    assert len(original) == provenance["source_size_bytes"] == 588
    digest = hashlib.sha256(original).hexdigest()
    blob = hashlib.sha1(b"blob " + str(len(original)).encode() + b"\0" + original).hexdigest()
    assert digest == provenance["source_sha256"] == (
        "d636ca5e201b2bed6596eba50aad651f35f260ef3504ad5add78ecac2c5e6d89")
    assert blob == provenance["source_git_blob"] == "9fef242e5ac3fb4bea696b139e3f6f14fab51d0c"
    assert provenance["source_commit"] == "cc265b5dd849af219f1f94f6a28e35cf9ce24719"
    assert provenance["source_repository"] == "Wego-Robot-Engeering-Team/corners"
    assert provenance["source_url"] == (
        "https://github.com/" + provenance["source_repository"] + "/blob/" +
        provenance["source_commit"] + "/" + provenance["source_path"])
    assert provenance["camera_side"] == "front"
    assert provenance["image_size"] == [1920, 1080]
    assert provenance["metadata_changes"] == {
        "camera_name": {"source": "b2_camera", "profile": "b2_front_camera"}}


def test_profile_retains_measured_fisheye_skew_and_raw_projection():
    document = yaml.safe_load((CALIBRATION / "b2_front_1920x1080.yaml").read_text())
    assert (document["image_width"], document["image_height"]) == (1920, 1080)
    assert document["distortion_model"] == "equidistant"
    assert document["camera_name"] == "b2_front_camera"
    assert document["distortion_coefficients"]["data"] == [0.387385, 0.107582, -0.100004, -0.01427]
    intrinsic = document["camera_matrix"]["data"]
    projection = document["projection_matrix"]["data"]
    assert intrinsic == [709.03283, 5.18641, 960.18835, 0.0, 719.53793, 506.59977, 0.0, 0.0, 1.0]
    assert document["rectification_matrix"]["data"] == [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    assert projection == [*intrinsic[:3], 0.0, *intrinsic[3:6], 0.0, *intrinsic[6:], 0.0]
    # A rear output template is allowed, but it must contain no copied front
    # measurement and cannot be used as an executable calibration profile.
    back = yaml.safe_load((CALIBRATION / "b2_back_camera.example.yaml").read_text())
    assert back["camera_name"] == "b2_back_camera"
    for key in ("camera_matrix", "distortion_coefficients", "rectification_matrix", "projection_matrix"):
        assert all(value is None for value in back[key]["data"])


def test_archived_corners_results_preserve_bytes_and_match_the_front_profile():
    archive = CALIBRATION / "corners"
    manifest = json.loads((archive / "manifest.json").read_text())
    assert manifest["repository"] == "Wego-Robot-Engeering-Team/corners"
    assert manifest["commit"] == "cc265b5dd849af219f1f94f6a28e35cf9ce24719"
    assert manifest["rear_calibration_found"] is False
    assert {entry["archive_file"] for entry in manifest["files"]} == {
        "b2_camera_info_raw.yaml", "b2_camera_info_rectified.yaml", "ost.yaml", "ost.txt", "260402_1312.txt"}
    for entry in manifest["files"]:
        payload = (archive / entry["archive_file"]).read_bytes()
        assert len(payload) == entry["size_bytes"]
        assert hashlib.sha256(payload).hexdigest() == entry["sha256"]
        assert hashlib.sha1(b"blob " + str(len(payload)).encode() + b"\0" + payload).hexdigest() == entry["git_blob"]
    raw = yaml.safe_load((archive / "b2_camera_info_raw.yaml").read_text())
    calibration = yaml.safe_load((archive / "ost.yaml").read_text())
    profile = yaml.safe_load((CALIBRATION / "b2_front_1920x1080.yaml").read_text())
    for key in ("image_width", "image_height", "distortion_model", "camera_matrix",
                "distortion_coefficients", "rectification_matrix", "projection_matrix"):
        assert raw[key] == calibration[key] == profile[key]
    assert raw["camera_name"] == "b2_camera" and profile["camera_name"] == "b2_front_camera"
