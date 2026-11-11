"""Exercise the real generated SDK API without a hardware driver."""
import os, signal, subprocess, sys, time, unittest
os.environ['ROS_DOMAIN_ID']=str(170+os.getpid()%20)
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from std_srvs.srv import Trigger
from b2_interface.srv import SetMotionMode
ADAPTER=sys.argv.pop(1)
class PostureRuntime(unittest.TestCase):
    def setUp(self):
        rclpy.init(); self.node=rclpy.create_node('test_posture_sdk'); self.requests=[]
        self.release=True; self.failure=False
        async def handle(request,response):
            self.requests.append(request.mode); done=Future()
            def check():
                if self.release and not done.done(): done.set_result(True)
            timer=self.node.create_timer(.02,check,callback_group=self.group)
            try: await done
            finally: self.node.destroy_timer(timer)
            response.success=not self.failure; response.result_code=12 if self.failure else 0
            response.message='sdk rejected' if self.failure else 'sdk accepted'; return response
        self.group=ReentrantCallbackGroup()
        self.service=self.node.create_service(SetMotionMode,'/b2/motion/set_mode',handle,callback_group=self.group)
        self.process=subprocess.Popen([ADAPTER,'--ros-args','-p','response_timeout:=0.3'],stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT,
            env={**os.environ, "RMW_IMPLEMENTATION": os.environ.get("POSTURE_TEST_RMW", os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"))})
    def until(self,predicate,timeout=4):
        end=time.monotonic()+timeout
        while not predicate() and time.monotonic()<end: rclpy.spin_once(self.node,timeout_sec=.01)
        self.assertTrue(predicate())
    def request(self,name):
        client=self.node.create_client(Trigger,'/'+name); self.assertTrue(client.wait_for_service(timeout_sec=4))
        return client.call_async(Trigger.Request())
    def test_all_postures_and_sdk_errors(self):
        for name,mode in [('stand_up',2),('stand_down',3),('balance_stand',1),('recovery_stand',4),('damp',0),('b2/stairs/enter',SetMotionMode.Request.VISION_WALK_ON),('b2/stairs/exit',SetMotionMode.Request.CLASSIC_WALK_ON)]:
            before=len(self.requests); future=self.request(name); self.until(future.done)
            self.assertTrue(future.result().success,future.result().message)
            self.assertEqual(self.requests[before:],[mode])
        self.failure=True; future=self.request('stand_up'); self.until(future.done)
        self.assertFalse(future.result().success); self.assertIn('12',future.result().message)
    def test_timeout_blocks_overlapping_retries(self):
        self.release=False; first=self.request('stand_up'); self.until(first.done)
        self.assertFalse(first.result().success); self.assertIn('unknown',first.result().message)
        retry=self.request('stand_down'); self.until(retry.done)
        self.assertFalse(retry.result().success); self.assertEqual(self.requests,[2])
        self.release=True; end=time.monotonic()+.3
        while time.monotonic()<end: rclpy.spin_once(self.node,timeout_sec=.01)
        future=self.request('stand_down'); self.until(future.done)
        self.assertTrue(future.result().success); self.assertEqual(self.requests,[2,3])
    def test_no_backend_does_not_claim_success(self):
        self.node.destroy_service(self.service)
        future=self.request('balance_stand'); self.until(future.done)
        self.assertFalse(future.result().success); self.assertEqual(self.requests,[])
    def tearDown(self):
        self.process.send_signal(signal.SIGINT)
        try: self.process.wait(timeout=3)
        except subprocess.TimeoutExpired: self.process.kill(); self.process.wait()
        self.node.destroy_node(); rclpy.shutdown()
if __name__=='__main__': unittest.main()
