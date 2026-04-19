class ActuatorFault:
    def __init__():
        pass

class SteeringFault(ActuatorFault):
    def __init__(self):
        super().__init__()
        self.steering_bias = 0

    def inject(self, true_steering):
        """Add a bias to the true steering angle."""
        # if self.steering_bias == 0:
        #     # Randomly determine a steering bias between -15 and +15 degrees
        #     self.steering_bias = np.random.uniform(-15, 15)
        #     print(f"Generated steering bias: {self.steering_bias:.2f} degrees")
        # return true_steering + self.steering_bias
        print("Injecting extreme bias into steering angle...")
        return true_steering + 100  # Extreme bias for testing purposes