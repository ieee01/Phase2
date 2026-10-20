import unittest
from cstam_core.delivery_task_manager import TaskQueueManager
from cstam_core.docking_controller import AutoDockingController

class TestCstamCoreNodes(unittest.TestCase):

    def test_task_queue_manager(self):
        mgr = TaskQueueManager()
        
        # Test request addition
        res = mgr.add_delivery_request("Table 1", "Soup & Water")
        self.assertTrue(res["success"])
        self.assertEqual(len(mgr.queue), 1)

        # Test case-insensitive resolution
        res_case = mgr.add_delivery_request("table 2", "Coffee")
        self.assertTrue(res_case["success"])
        self.assertEqual(res_case["task"]["target"], "Table 2")

        # Test unknown location error
        res_err = mgr.add_delivery_request("Table 99", "Coffee")
        self.assertFalse(res_err["success"])

        # Test task retrieval
        task = mgr.get_next_task()
        self.assertIsNotNone(task)
        self.assertEqual(task["target"], "Table 1")
        self.assertEqual(task["status"], "en_route")

        # Test task completion
        completed = mgr.complete_current_task(success=True)
        self.assertEqual(completed["status"], "completed")

    def test_auto_docking_controller(self):
        ctrl = AutoDockingController(idle_timeout=5.0)
        
        # Normal state (busy or queue not empty)
        res = ctrl.evaluate_dock_trigger(queue_empty=False, is_navigating=True, current_time=100.0)
        self.assertEqual(res, 'none')

        # Idle timeout trigger
        res_idle_1 = ctrl.evaluate_dock_trigger(queue_empty=True, is_navigating=False, current_time=100.0)
        self.assertEqual(res_idle_1, 'none')
        res_idle_2 = ctrl.evaluate_dock_trigger(queue_empty=True, is_navigating=False, current_time=106.0)
        self.assertEqual(res_idle_2, 'dock_idle')


if __name__ == '__main__':
    unittest.main()
