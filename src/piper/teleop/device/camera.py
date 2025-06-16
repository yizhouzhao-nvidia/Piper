import cv2
import mediapipe as mp
import numpy as np
import click
from typing import Tuple, List, Optional, Dict
from contextlib import contextmanager

class HandTracker:
    def __init__(self, 
                 static_image_mode: bool = False,
                 max_num_hands: int = 1,
                 min_detection_confidence: float = 0.2,
                 min_tracking_confidence: float = 0.5):
        """
        Initialize the hand tracker with MediaPipe.
        
        Args:
            static_image_mode: If True, treats input as a single image, not a video stream
            max_num_hands: Maximum number of hands to detect
            min_detection_confidence: Minimum confidence for hand detection
            min_tracking_confidence: Minimum confidence for hand tracking
        """
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=static_image_mode,
            max_num_hands=max_num_hands,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence
        )
        self.mp_draw = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles

    def process_frame(self, frame: np.ndarray) -> Tuple[np.ndarray, Optional[List[dict]]]:
        """
        Process a single frame to detect hands and their landmarks.
        
        Args:
            frame: Input frame (BGR format)
            
        Returns:
            Tuple containing:
            - Processed frame with hand landmarks drawn
            - List of dictionaries containing hand information (None if no hands detected)
        """
        # Convert BGR to RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Process the frame
        results = self.hands.process(frame_rgb)
        
        # Convert back to BGR for display
        frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        
        hand_info = []
        
        if results.multi_hand_landmarks:
            for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):

                # Draw landmarks on the frame
                self.mp_draw.draw_landmarks(
                    frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style()
                )
                
                # Extract hand information
                landmarks = []
                for landmark in hand_landmarks.landmark:
                    landmarks.append({
                        'x': landmark.x,
                        'y': landmark.y,
                        'z': landmark.z,
                        'visibility': landmark.visibility
                    })
                    
                
                hand_info.append({
                    'landmarks': landmarks,
                    'handedness': handedness.classification[0].label,
                    'confidence': handedness.classification[0].score
                })
        
        return frame, hand_info if hand_info else None

    def get_finger_positions(self, hand_info: List[dict]) -> List[dict]:
        """
        Calculate finger positions based on hand landmarks.
        
        Args:
            hand_info: List of dictionaries containing hand information
            
        Returns:
            List of dictionaries containing finger positions for each detected hand
        """
        finger_positions = []
        
        for hand in hand_info:
            landmarks = hand['landmarks']
            
            # Define finger tip and pip indices
            finger_indices = {
                'thumb': {'tip': 4, 'pip': 2},
                'index': {'tip': 8, 'pip': 6},
                'middle': {'tip': 12, 'pip': 10},
                'ring': {'tip': 16, 'pip': 14},
                'pinky': {'tip': 20, 'pip': 18}
            }
            
            fingers = {}
            for finger_name, indices in finger_indices.items():
                tip = landmarks[indices['tip']]
                pip = landmarks[indices['pip']]
                
                # use the distance between tip and pip to determine if the finger is extended
                dist = np.linalg.norm(np.array([tip['x'], tip['y']]) - np.array([pip['x'], pip['y']])) 
                # print(f'{finger_name} distance: {dist}')
                # FIXME: this distance doesn't work for the thumb
                is_extended = dist > 0.07
                
                fingers[finger_name] = {
                    'position': {
                        'x': tip['x'],
                        'y': tip['y'],
                        'z': tip['z']
                    },
                    'is_extended': is_extended
                }
            
            finger_positions.append({
                'handedness': hand['handedness'],
                'fingers': fingers
            })
        
        return finger_positions

    def get_wrist_pose(self, hand_info: List[dict]) -> List[Dict]:
        """
        Calculate wrist position and rotation based on hand landmarks.
        
        Args:
            hand_info: List of dictionaries containing hand information
            
        Returns:
            List of dictionaries containing wrist pose information for each detected hand
        """
        wrist_poses = []
        
        for hand in hand_info:
            landmarks = hand['landmarks']
            
            # Wrist is landmark 0
            wrist = landmarks[0]

            # set wrist z and average of landmarks z from indexes: 5, 9, 13, 17
            wrist['z'] = np.log(np.abs(np.mean([landmarks[5]['z'], landmarks[9]['z'], landmarks[13]['z'], landmarks[17]['z']])))

            # if np.random.rand() < 0.05:
            #     # print the wrist position in 3 decimal places
            #     print(f'Wrist position: (x: {wrist["x"]*100:.3f}, y: {wrist["y"]*100:.3f}, z: {wrist["z"]*100:.3f})')
            
            # Get index finger MCP (knuckle) for rotation calculation
            middle_mcp = landmarks[9]
            
            # Calculate rotation using wrist and index finger MCP
            # This gives us a rough estimate of hand orientation
            dx = middle_mcp['x'] - wrist['x']
            dy = middle_mcp['y'] - wrist['y']
            dz = middle_mcp['z'] - wrist['z']



            # if np.random.rand() < 0.05:
            #     idx = self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP
            #     print(
            #         idx,
            #         f'coordinates: (',
            #         f'x: {landmarks[idx]["x"] * 100:.3f}, ',
            #         f'y: {landmarks[idx]["y"] * 100:.3f}, ',
            #         f'z: {landmarks[idx]["z"] * 100:.3f}'
            #     )

            #     idx = self.mp_hands.HandLandmark.THUMB_MCP
            #     print(
            #         idx,
            #         f'coordinates: (',
            #         f'x: {landmarks[idx]["x"] * 100:.3f}, ',
            #         f'y: {landmarks[idx]["y"] * 100:.3f}, ',
            #         f'z: {landmarks[idx]["z"] * 100:.3f}'
            #     )
        
            # Calculate yaw (rotation around z-axis)
            yaw = np.arctan2(dy, dx)
            
            # if np.random.rand() < 0.05:
            #     print(f'Yaw: {np.degrees(yaw):.3f}', dx, dy, dz)

            # Calculate pitch (rotation around y-axis)
            pitch = np.arctan2(dz, np.sqrt(dx*dx + dy*dy))
            
            # Calculate roll (rotation around x-axis)
            # Use middle finger MCP for roll calculation
            middle_mcp = landmarks[9]
            roll = np.arctan2(middle_mcp['y'] - wrist['y'], middle_mcp['z'] - wrist['z'])
            
            wrist_poses.append({
                'handedness': hand['handedness'],
                'position': {
                    'x': wrist['x'],
                    'y': wrist['y'],
                    'z': wrist['z'] 
                },
                'rotation': {
                    'roll': 0,
                    'pitch': 0,
                    'yaw': 0
                }
            })
        
        return wrist_poses

    def release(self):
        """Release resources."""
        self.hands.close()

@contextmanager
def camera(device: int):
    """Context manager for camera."""
    cap = cv2.VideoCapture(device)
    tracker = HandTracker()
    yield cap, tracker
    cap.release()
    tracker.release()
    cv2.destroyAllWindows()

@click.command()
@click.option('--device', type=int, default=4, help='Camera device index')
def main(device):
    """Example usage of the HandTracker class."""
    # Initialize camera
    with camera(device) as (cap, tracker):
        while True:
            ret, frame = cap.read()
            if not ret:
                break
                
            # Process frame
            processed_frame, hand_info = tracker.process_frame(frame)
            
            if hand_info:
                # Get finger positions
                finger_positions = tracker.get_finger_positions(hand_info)
                
                # Get wrist poses
                wrist_poses = tracker.get_wrist_pose(hand_info)
                
                # Print information for the first detected hand
                if finger_positions and wrist_poses:
                    print("\nHand Information:")
                    print(f"Handedness: {wrist_poses[0]['handedness']}")
                    
                    print("\nWrist Position:")
                    pos = wrist_poses[0]['position']
                    print(f"X: {pos['x']:.3f}, Y: {pos['y']:.3f}, Z: {pos['z']:.3f}")
                    
                    # print("\nWrist Rotation (degrees):")
                    # rot = wrist_poses[0]['rotation']
                    # print(f"Roll: {rot['roll']:.1f}, Pitch: {rot['pitch']:.1f}, Yaw: {rot['yaw']:.1f}")
                    
                    # print("\nFinger States:")
                    # for finger_name, finger_data in finger_positions[0]['fingers'].items():
                    #     print(f"{finger_name}: {'Extended' if finger_data['is_extended'] else 'Bent'}")
            
            # Display the frame
            cv2.imshow('Hand Tracking', processed_frame)
            
            # Break loop on 'q' press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
                

if __name__ == "__main__":
    main()
