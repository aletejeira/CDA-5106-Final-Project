import os
import math
import yaml
import lmdb
import numpy as np
import torch
import wandb
import carla
import random
import string
import cv2

from torch.distributions.categorical import Categorical

from leaderboard_codes.autonomous_agent1 import AutonomousAgent, Track
from utils import visualize_obs

from rails.models import EgoModel, CameraModel
from waypointer import Waypointer

from fault_injector import FaultInjector


def get_entry_point():
    return 'ImageAgent'

class ImageAgent(AutonomousAgent):
    
    """
    Trained image agent
    """
    
    def setup(self, path_to_conf_file):
        """
        Setup the agent parameters
        """

        self.track = Track.SENSORS
        self.num_frames = 0
        self.video_writer = None
        self.video_path = os.path.abspath('camera_view.mp4')

        config_dir = os.path.dirname(os.path.abspath(path_to_conf_file))

        with open(path_to_conf_file, 'r') as f:
            config = yaml.safe_load(f)

        # Loop through the items and correct the paths
        for key, value in config.items():
            if key.endswith('_dir'):
                # The paths in your YAML file are relative to the PCLA folder.
                # We need to go up two directories from the config folder to get to PCLA.
                
                project_root = os.path.dirname(os.path.dirname(config_dir))
                
                # Construct the absolute path by joining the project root and the relative path from the YAML
                absolute_path = os.path.join(project_root, value)
                
                # Now, set the attribute with the corrected absolute path
                setattr(self, key, absolute_path)
            else:
                # For non-path values, set them as is
                setattr(self, key, value)

        self.device = torch.device('cuda')

        self.image_model = CameraModel(config).to(self.device)
        self.image_model.load_state_dict(torch.load(self.main_model_dir))
        self.image_model.eval()

        self.vizs = []

        self.waypointer = None

        if self.log_wandb:
            wandb.init(project='carla_evaluate')
            
        self.steers = torch.tensor(np.linspace(-self.max_steers,self.max_steers,self.num_steers)).float().to(self.device)
        self.throts = torch.tensor(np.linspace(0,self.max_throts,self.num_throts)).float().to(self.device)

        self.prev_steer = 0
        self.lane_change_counter = 0
        self.stop_counter = 0

    def destroy(self):
        # Release video writer
        if self.video_writer is not None:
            self.video_writer.release()
            print(f"Camera video saved to {self.video_path}")
        
        if len(self.vizs) == 0:
            return

        self.flush_data()
        self.prev_steer = 0
        self.lane_change_counter = 0
        self.stop_counter = 0
        self.lane_changed = None
        
        del self.waypointer
        del self.image_model
    
    def flush_data(self):

        if self.log_wandb:
            wandb.log({
                'vid': wandb.Video(np.stack(self.vizs).transpose((0,3,1,2)), fps=20, format='mp4')
            })
            
        self.vizs.clear()

    def sensors(self):
        sensors = [ # Sensors modified to match online leaderboard based on https://github.com/dotchen/WorldOnRails/issues/27
            {'type': 'sensor.speedometer', 'id': 'EGO'},
            {'type': 'sensor.other.gnss', 'x': 0., 'y': 0.0, 'z': self.camera_z, 'id': 'GPS'},
            {'type': 'sensor.camera.rgb', 'x': self.camera_x, 'y': 0, 'z': self.camera_z, 'roll': 0.0, 'pitch': 0.0, 'yaw': -55.0,
            'width': 160, 'height': 240, 'fov': 60, 'id': f'Wide_RGB_0'},
            {'type': 'sensor.camera.rgb', 'x': self.camera_x, 'y': 0, 'z': self.camera_z, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            'width': 160, 'height': 240, 'fov': 60, 'id': f'Wide_RGB_1'},
            {'type': 'sensor.camera.rgb', 'x': self.camera_x, 'y': 0, 'z': self.camera_z, 'roll': 0.0, 'pitch': 0.0, 'yaw':  55.0,
            'width': 160, 'height': 240, 'fov': 60, 'id': f'Wide_RGB_2'},
            {'type': 'sensor.camera.rgb', 'x': self.camera_x, 'y': 0, 'z': self.camera_z, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            'width': 384, 'height': 240, 'fov': 50, 'id': f'Narrow_RGB'},
        ]
        return sensors

    def run_step(self, input_data, timestamp, vehicle=None):
        # changed to match the online leaderboard based on https://github.com/dotchen/WorldOnRails/issues/27
        wide_rgbs = []
        for i in range(3):
            _, wide_rgb = input_data.get(f'Wide_RGB_{i}')

            wide_rgb_crop = wide_rgb[self.wide_crop_top:,:,:3]

            # # Optional fault injection: if main attaches `self.rectangle_mask_fault`,
            # # apply it to the center wide camera (Wide_RGB_1).
            # if i == 1 and getattr(self, 'rectangle_mask_fault', None) is not None:
            #     wide_rgb_crop = self.rectangle_mask_fault.inject(wide_rgb_crop)
            if (
                i == 1
                and getattr(self, 'rectangle_mask_fault', None) is not None
                and timestamp >= 5.0
            ):
                wide_rgb_crop = self.rectangle_mask_fault.inject(wide_rgb_crop)
            #########################################################################
            if i == 1 and getattr(self, 'gaussian_noise_fault', None) is not None and timestamp >= 5.0:
                wide_rgb_crop = self.gaussian_noise_fault.inject(wide_rgb_crop)
            ############################################################################

            # BLACKOUT INJECTION FAULT: The blackout starts after 5 seconds, lasts for 10 seconds, then is camera goes back to normal.
            # if (i == 1 and getattr(self, 'blackout_fault', None) is not None and timestamp >= 5.0 and timestamp < 15.0):
            #     wide_rgb_crop = self.blackout_fault.inject(wide_rgb_crop)

            # BLACKOUT INJECTION FAULT: The blackout starts after 5 seconds, lasts for 5 seconds, then is camera goes back to normal for 5 seconds, blackout again. Repeat for the rest of experiment.
           # 5s delay, then 5s blackout / 5s normal repeating
            start_time = 5.0
            blackout_on = 5.0
            blackout_off = 5.0
            cycle = blackout_on + blackout_off

            t = timestamp - start_time
            blackout_active = (t >= 0.0) and ((t % cycle) < blackout_on)

            if i == 1 and getattr(self, 'blackout_fault', None) is not None and blackout_active:
                wide_rgb_crop = self.blackout_fault.inject(wide_rgb_crop)



            _wide_rgb = wide_rgb_crop[...,::-1].copy()
            wide_rgbs.append(_wide_rgb)

        wide_rgbs_con = np.concatenate([wide_rgbs[0],wide_rgbs[1],wide_rgbs[2]], axis=1)

        if self.video_writer is None:
            h, w = wide_rgbs_con.shape[:2]
            self.video_writer = cv2.VideoWriter(
                self.video_path,
                cv2.VideoWriter_fourcc(*'mp4v'),
                20,
                (w, h),
            )

        if self.video_writer is not None and self.video_writer.isOpened():
            frame_bgr = cv2.cvtColor(wide_rgbs_con, cv2.COLOR_RGB2BGR)
            self.video_writer.write(frame_bgr)

        wide_rgbs_ = torch.tensor(wide_rgbs_con[None]).float().permute(0,3,1,2).to(self.device)
        
        
        _, narr_rgb = input_data.get(f'Narrow_RGB')
        narr_rgb_crop = narr_rgb[:-self.narr_crop_bottom,:,:3]
        _narr_rgb = narr_rgb_crop[...,::-1].copy()

        # Crop images
        #_wide_rgb = wide_rgb[self.wide_crop_top:,:,:3]
        #_narr_rgb = narr_rgb[:-self.narr_crop_bottom,:,:3]

        #wide_rgb = _wide_rgb[...,::-1].copy()
        #_narr_rgb = _narr_rgb[...,::-1].copy()

        _, ego = input_data.get('EGO')
        _, gps = input_data.get('GPS')

        # # Inject GNSS drift fault (Does this go here?)
        # if getattr(self, 'gnss_drift_fault', None) is not None and timestamp >= 5.0:
        #     gps = self.gnss_drift_fault.inject(gps)


        if self.waypointer is None:
            self.waypointer = Waypointer(self._global_plan, gps)

        _, _, cmd = self.waypointer.tick(gps)

        spd = ego.get('speed')

        # # Inject Speedometer bias fault
        # if hasattr(self, 'speed_bias_fault') and timestamp >= 5.0:
        #     spd = self.speed_bias_fault.inject(spd)

        cmd_value = cmd.value-1
        cmd_value = 3 if cmd_value < 0 else cmd_value

        if cmd_value in [4,5]:
            if self.lane_changed is not None and cmd_value != self.lane_changed:
                self.lane_change_counter = 0

            self.lane_change_counter += 1
            self.lane_changed = cmd_value if self.lane_change_counter > {4:200,5:200}.get(cmd_value) else None
        else:
            self.lane_change_counter = 0
            self.lane_changed = None

        if cmd_value == self.lane_changed:
            cmd_value = 3

        _wide_rgb = torch.tensor(_wide_rgb[None]).float().permute(0,3,1,2).to(self.device)
        _narr_rgb = torch.tensor(_narr_rgb[None]).float().permute(0,3,1,2).to(self.device)
        
        if self.all_speeds:
            steer_logits, throt_logits, brake_logits = self.image_model.policy(wide_rgbs_, _narr_rgb, cmd_value)
            #steer_logits, throt_logits, brake_logits = self.image_model.policy(_wide_rgb, _narr_rgb, cmd_value)
            # Interpolate logits
            steer_logit = self._lerp(steer_logits, spd)
            throt_logit = self._lerp(throt_logits, spd)
            brake_logit = self._lerp(brake_logits, spd)
        else:
            steer_logit, throt_logit, brake_logit = self.image_model.policy(_wide_rgb, _narr_rgb, cmd_value, spd=torch.tensor([spd]).float().to(self.device))

        
        action_prob = self.action_prob(steer_logit, throt_logit, brake_logit)

        brake_prob = float(action_prob[-1])

        steer = float(self.steers @ torch.softmax(steer_logit, dim=0))
        throt = float(self.throts @ torch.softmax(throt_logit, dim=0))

        steer, throt, brake = self.post_process(steer, throt, brake_prob, spd, cmd_value)

        #rgb = np.concatenate([wide_rgb, narr_rgb[...,:3]], axis=1)
        
        #self.vizs.append(visualize_obs(rgb, 0, (steer, throt, brake), spd, cmd=cmd_value+1))


        if len(self.vizs) > 1000:
            self.flush_data()

        self.num_frames += 1

        return carla.VehicleControl(steer=steer, throttle=throt, brake=brake)
    
    def _lerp(self, v, x):
        D = v.shape[0]

        min_val = self.min_speeds
        max_val = self.max_speeds

        x = (x - min_val)/(max_val - min_val)*(D-1)

        x0, x1 = max(min(math.floor(x), D-1),0), max(min(math.ceil(x), D-1),0)
        w = x - x0

        return (1-w) * v[x0] + w * v[x1]

    def action_prob(self, steer_logit, throt_logit, brake_logit):

        steer_logit = steer_logit.repeat(self.num_throts)
        throt_logit = throt_logit.repeat_interleave(self.num_steers)

        action_logit = torch.cat([steer_logit, throt_logit, brake_logit[None]])

        return torch.softmax(action_logit, dim=0)

    def post_process(self, steer, throt, brake_prob, spd, cmd):
        
        if brake_prob > 0.5:
            steer, throt, brake = 0, 0, 1
        else:
            brake = 0
            throt = max(0.4, throt)

        # # To compensate for non-linearity of throttle<->acceleration
        # if throt > 0.1 and throt < 0.4:
        #     throt = 0.4
        # elif throt < 0.1 and brake_prob > 0.3:
        #     brake = 1

        if spd > {0:10,1:10}.get(cmd, 20)/3.6: # 10 km/h for turning, 15km/h elsewhere
            throt = 0

        # if cmd == 2:
        #     steer = min(max(steer, -0.2), 0.2)

        # if cmd in [4,5]:
        #     steer = min(max(steer, -0.4), 0.4) # no crazy steerings when lane changing

        return steer, throt, brake
    
def load_state_dict(model, path):

    from collections import OrderedDict
    new_state_dict = OrderedDict()
    state_dict = torch.load(path)
    
    for k, v in state_dict.items():
        name = k[7:] # remove `module.`
        new_state_dict[name] = v
    
    model.load_state_dict(new_state_dict)
