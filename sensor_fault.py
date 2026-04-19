import numpy as np

class SensorFault:
    def __init__(self):
        pass
    def inject(self):
        pass


class CameraFault(SensorFault):
    def __init__(self, frequency, duration):
        self.x = 0
        self.y = 0
        self.width = 0
        self.height = 0
        self.frequency = frequency
        self.duration = duration
        self.inject_counter = 0

    def inject(self):
        pass

class Blackout(CameraFault):
    def __init__(self):
        super().__init__(frequency=10, duration=10)  # Example frequency and duration, can be parameterized
        pass
    def inject(self, InputImage):
        """Black out the entire provided frame."""
        InputImage[:, :, :] = 0
        return InputImage


class Occlusion(CameraFault):
    def __init__(self):
        super().__init__(frequency=10, duration=10)  # Example frequency and duration, can be parameterized
        pass
    
    def inject(self):
        self.inject_counter += 1

class RectangleMask(Occlusion):
    def __init__(self):
        super().__init__()
        self._mask_ready = False

    # Randomly determine the size and position of the rectangle mask
    def randomly_generate_mask_dimensions(self, frame_shape=None):
        print("Generating random rectangle mask dimensions...")
        # Default to the WoR Wide_RGB_* resolution if no frame is provided.
        if frame_shape is None:
            h, w = 240, 160
        else:
            h, w = int(frame_shape[0]), int(frame_shape[1])

        self.width = int(np.random.randint(max(1, int(0.10 * w)), max(2, int(0.40 * w)) + 1))
        self.height = int(np.random.randint(max(1, int(0.10 * h)), max(2, int(0.40 * h)) + 1))
        self.x = int(np.random.randint(0, max(1, w - self.width + 1)))
        self.y = int(np.random.randint(0, max(1, h - self.height + 1)))
        self._mask_ready = True
        return
    def generate_center_positioned_mask(self, frame_shape=None):
        print("Generating center-positioned rectangle mask dimensions...")
        # Default to the WoR Wide_RGB_* resolution if no frame is provided.
        if frame_shape is None:
            h, w = 240, 160
        else:
            h, w = int(frame_shape[0]), int(frame_shape[1])

        self.width = int(0.20 * w)  # Fixed size for center mask (20% of width)
        self.height = int(0.20 * h)  # Fixed size for center mask (20% of height)
        self.x = int((w - self.width) / 2)  # Center horizontally
        self.y = int((h - self.height) / 2)  # Center vertically
        self._mask_ready = True
        return
    def generate_full_frame_mask(self, frame_shape=None):
        print("Generating full-frame rectangle mask dimensions...")
        # Default to the WoR Wide_RGB_* resolution if no frame is provided.
        if frame_shape is None:
            h, w = 240, 160
        else:
            h, w = int(frame_shape[0]), int(frame_shape[1])

        self.width = w
        self.height = h
        self.x = 0
        self.y = 0
        self._mask_ready = True
        return
    def generate_right_half_mask(self, frame_shape=None):
        print("Generating right-half rectangle mask dimensions...")
        # Default to the WoR Wide_RGB_* resolution if no frame is provided.
        if frame_shape is None:
            h, w = 240, 160
        else:
            h, w = int(frame_shape[0]), int(frame_shape[1])

        self.width = int(w / 2)
        self.height = h
        self.x = int(w / 2)
        self.y = 0
        self._mask_ready = True
        return

    # Apply the rectangle mask to the agent's camera sensor
    def inject(self, InputImage):
        """Black out a fixed rectangle on the provided frame.
        """
        if not self._mask_ready:
            # self.randomly_generate_mask_dimensions(frame_shape=InputImage.shape)
            self.generate_center_positioned_mask(frame_shape=InputImage.shape)
            # self.generate_full_frame_mask(frame_shape=InputImage.shape)
            # self.generate_right_half_mask(frame_shape=InputImage.shape)
        InputImage[self.y:self.y+self.height,self.x:self.x+self.width,:] = 0
        return InputImage
        

class TransparentRectangleMask(RectangleMask):
    def __init__(self):
        super().__init__()
    



class Noise(CameraFault):
    def __init__(self):
        pass

class Gaussian(Noise):
    def __init__(self):
        pass

    def inject(self, InputImage):
        """Add Gaussian noise to the provided frame.
        """
        # print("Injecting Gaussian noise into the image...")
        mean = 0
        stddev = 25  # Adjust the standard deviation for more or less noise
        gaussian_noise = np.random.normal(mean, stddev, InputImage.shape).astype(np.float32)
        noisy_image = InputImage.astype(np.float32) + gaussian_noise
        noisy_image = np.clip(noisy_image, 0, 255).astype(np.uint8)  # Ensure pixel values are valid
        return noisy_image



class SaltAndPepper(Noise):
    def __init__(self):
        pass

class SpeedometerFault(SensorFault):
    def __init__(self):
        pass
    
class SpeedometerBias(SpeedometerFault):
    def __init__(self):
        super().__init__()
        self.bias_value = 0

    def inject(self, true_speed):
        """Add a bias to the true speed value."""
        # if self.bias_value == 0:
        #     # Randomly determine a bias value between -10 and +10 km/h
        #     self.bias_value = np.random.uniform(-10, 10)
        #     print(f"Generated bias value: {self.bias_value:.2f} km/h")
        # return true_speed + self.bias_value
        print("Injecting extreme bias into speedometer reading...")
        return true_speed + 100  # Extreme bias for testing purposes

class GNSSFault(SensorFault):
    def __init__(self):
        pass

class GNSSDrift(GNSSFault):
    def __init__(self):
        super().__init__()
        self.drift_value = 0

    def inject(self, true_gnss):
        """Add a drift to the true GNSS coordinates."""
        # if self.drift_value == 0:
        #     # Randomly determine a drift value (e.g., up to 50 meters)
        #     self.drift_value = np.random.uniform(-50, 50)
        #     print(f"Generated GNSS drift value: {self.drift_value:.2f} meters")
        # # Convert drift from meters to degrees (approximate)
        # drift_in_degrees = self.drift_value / 111320  # Rough conversion for latitude
        # return (true_gnss[0] + drift_in_degrees, true_gnss[1] + drift_in_degrees, true_gnss[2])
        print("Injecting extreme drift into GNSS reading...")
        return (true_gnss[0] + 1.0, true_gnss[1] + 1.0, true_gnss[2])  # Extreme drift for testing purposes

# Mayve we will do this? Is this the actuator fault?
# class SteeringFault(): 