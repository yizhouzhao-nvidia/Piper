from typing import Union, Optional
import numbers
import numpy as np
import scipy.interpolate as si

class PoseTrajectoryInterpolator:
    def __init__(self, times: np.ndarray, poses: np.ndarray):
        assert len(times) >= 1
        assert len(poses) == len(times)

        times = np.asarray(times)
        poses = np.asarray(poses)

        self.num_joint = len(poses[0])

        if len(times) == 1:
            # special treatment for single step interpolation
            self.single_step = True
            self._times = times
            self._poses = poses
        else:
            self.single_step = False
            assert np.all(times[1:] >= times[:-1])
            self.pose_interp = si.interp1d(times, poses, axis=0, assume_sorted=True)
    
    @property
    def times(self) -> np.ndarray:
        if self.single_step:
            return self._times
        else:
            return self.pose_interp.x
    
    @property
    def poses(self) -> np.ndarray:
        if self.single_step:
            return self._poses
        else:
            return self.pose_interp.y

    def trim(self, 
        start_t: float, end_t: float
    ) -> "PoseTrajectoryInterpolator":
        assert start_t <= end_t
        times = self.times
        should_keep = (start_t < times) & (times < end_t)
        keep_times = times[should_keep]
        all_times = np.concatenate([[start_t], keep_times, [end_t]])
        # remove duplicates, Slerp requires strictly increasing x
        all_times = np.unique(all_times)
        # interpolate
        all_poses = self(all_times)
        return PoseTrajectoryInterpolator(times=all_times, poses=all_poses)
    
    def drive_to_waypoint(self, 
            pose, time, curr_time,
            max_joint_speed=np.inf,
        ) -> "PoseTrajectoryInterpolator":
        if not isinstance(max_joint_speed, np.ndarray):
            max_joint_speed = np.array([max_joint_speed] * self.num_joint)
        
        assert len(max_joint_speed) == self.num_joint
        assert np.max(max_joint_speed) > 0
        time = max(time, curr_time)
        
        curr_pose = self(curr_time)
        pose_min_duration = np.max(np.abs(curr_pose - pose) / max_joint_speed)
        duration = time - curr_time
        duration = max(duration, pose_min_duration)
        assert duration >= 0
        last_waypoint_time = curr_time + duration

        # insert new pose
        trimmed_interp = self.trim(curr_time, curr_time)
        times = np.append(trimmed_interp.times, [last_waypoint_time], axis=0)
        poses = np.append(trimmed_interp.poses, [pose], axis=0)

        # create new interpolator
        final_interp = PoseTrajectoryInterpolator(times, poses)
        return final_interp

    def schedule_waypoint(self,
            pose, time, 
            max_joint_speed=np.inf,
            curr_time=None,
            last_waypoint_time=None
        ) -> "PoseTrajectoryInterpolator":
        if not isinstance(max_joint_speed, np.ndarray):
            max_joint_speed = np.array([max_joint_speed] * self.num_joint)
        
        assert len(max_joint_speed) == self.num_joint
        assert np.max(max_joint_speed) > 0

        if last_waypoint_time is not None:
            assert curr_time is not None

        # trim current interpolator to between curr_time and last_waypoint_time
        start_time = self.times[0]
        end_time = self.times[-1]
        assert start_time <= end_time
        if curr_time is not None:
            if time <= curr_time:
                # if insert time is earlier than current time
                # no effect should be done to the interpolator
                return self
            # now, curr_time < time
            start_time = max(curr_time, start_time)

            if last_waypoint_time is not None:
                # if last_waypoint_time is earlier than start_time
                # use start_time
                if time <= last_waypoint_time:
                    end_time = curr_time
                else:
                    end_time = max(last_waypoint_time, curr_time)
            else:
                end_time = curr_time

        end_time = min(end_time, time)
        start_time = min(start_time, end_time)
        # end time should be the latest of all times except time
        # after this we can assume order (proven by zhenjia, due to the 2 min operations)

        # Constraints:
        # start_time <= end_time <= time (proven by zhenjia)
        # curr_time <= start_time (proven by zhenjia)
        # curr_time <= time (proven by zhenjia)
        
        # time can't change
        # last_waypoint_time can't change
        # curr_time can't change
        assert start_time <= end_time
        assert end_time <= time
        if last_waypoint_time is not None:
            if time <= last_waypoint_time:
                assert end_time == curr_time
            else:
                assert end_time == max(last_waypoint_time, curr_time)

        if curr_time is not None:
            assert curr_time <= start_time
            assert curr_time <= time
        trimmed_interp = self.trim(start_time, end_time)
        # after this, all waypoints in trimmed_interp is within start_time and end_time
        # and is earlier than time

        # determine speed
        duration = time - end_time
        end_pose = trimmed_interp(end_time)
        pose_min_duration = np.max(np.abs(end_pose - pose) / max_joint_speed)
        duration = max(duration, pose_min_duration)
        assert duration >= 0
        last_waypoint_time = end_time + duration

        # insert new pose
        times = np.append(trimmed_interp.times, [last_waypoint_time], axis=0)
        poses = np.append(trimmed_interp.poses, [pose], axis=0)

        # create new interpolator
        final_interp = PoseTrajectoryInterpolator(times, poses)
        return final_interp


    def __call__(self, t: Union[numbers.Number, np.ndarray]) -> np.ndarray:
        is_single = False
        if isinstance(t, numbers.Number):
            is_single = True
            t = np.array([t])

        pose = np.zeros((len(t), self.num_joint))
        if self.single_step:
            pose[:] = self._poses[0]
        else:
            start_time = self.times[0]
            end_time = self.times[-1]
            t = np.clip(t, start_time, end_time)
            pose = self.pose_interp(t)
        
        if is_single:
            pose = pose[0]
        return pose
    

if __name__ == "__main__":
    # Create a simple trajectory with 3 waypoints
    times = np.array([0.0, 1.0, 2.0])
    # Each pose has 3 joints (for example)
    poses = np.array([
        [0.0, 0.0, 0.0],  # Initial pose
        [1.0, 0.5, 0.3],  # Middle pose
        [2.0, 1.0, 0.6]   # Final pose
    ])

    # Create the interpolator
    interpolator = PoseTrajectoryInterpolator(times, poses)

    # Test basic interpolation
    print("Testing basic interpolation:")
    test_times = np.array([0.0, 0.5, 1.0, 1.5, 2.0])
    interpolated_poses = interpolator(test_times)
    for t, pose in zip(test_times, interpolated_poses):
        print(f"Time {t:.1f}: {pose}")

    # Test trimming
    print("\nTesting trimming:")
    trimmed = interpolator.trim(0.5, 1.5)
    print(f"Trimmed times: {trimmed.times}")
    print(f"Trimmed poses: {trimmed.poses}")

    # Test driving to a new waypoint
    print("\nTesting drive_to_waypoint:")
    new_pose = np.array([3.0, 1.5, 0.9])
    new_time = 3.0
    curr_time = 1.0
    max_speed = 1.0  # Maximum speed for each joint
    new_trajectory = interpolator.drive_to_waypoint(
        new_pose, new_time, curr_time, max_speed
    )
    print(f"New trajectory times: {new_trajectory.times}")
    print(f"New trajectory poses: {new_trajectory.poses}")

    # Test scheduling a waypoint
    print("\nTesting schedule_waypoint:")
    scheduled = interpolator.schedule_waypoint(
        new_pose, new_time, max_speed, curr_time
    )
    print(f"Scheduled trajectory times: {scheduled.times}")
    print(f"Scheduled trajectory poses: {scheduled.poses}")
