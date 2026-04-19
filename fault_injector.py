# import numpy as np
import struct



class FaultInjector:
    def __init__(self, client=None):
        self.client = client
        self.evaluator = None

    def inject_fault(self, fault_object):
        """Call the fault object's inject method to apply the fault."""
        return fault_object.inject()
