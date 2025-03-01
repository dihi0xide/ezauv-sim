import math
import numpy as np
import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

import pygame
import subprocess

debug_text = "test test"
def set_text(text):
    global debug_text
    debug_text = text

class SimulationAnimator:
    def __init__(self, width=600, height=600, fps=30, frames=100, output_dir="videos"):
        self.width = width
        self.height = height
        self.scale_factor = self.width / 30  
        self.origin = np.array([width // 2, height // 2])  
        self.fps = fps
        self.frames = frames
        self.output_dir = output_dir
        self.frame_count = 0

        os.makedirs(output_dir, exist_ok=True)
        self.screen = pygame.display.set_mode((width, height))
        self.clock = pygame.time.Clock()

        self.ffmpeg_command = [
            "ffmpeg", "-y", "-r", str(self.fps),
            "-f", "rawvideo", "-pix_fmt", "rgb24",
            "-s", f"{self.width}x{self.height}",
            "-i", "-",  
            "-vcodec", "libx264", "-crf", "25", "-pix_fmt", "yuv420p",
            os.path.join(self.output_dir, "animation.mp4")
        ]
        pygame.font.init()
        self.font = pygame.font.SysFont('Arial', 30)


        self.ffmpeg_proc = subprocess.Popen(
            self.ffmpeg_command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )

    def to_screen(self, unit_position):
        return (unit_position * self.scale_factor + self.origin).astype(int)

    def append(self, position, rotation, velocity, motor_positions, motor_accelerations):
        self.screen.fill((255, 255, 255)) 

        angle = math.radians(rotation)
        cos_a, sin_a = np.cos(angle), np.sin(angle)

        square_points = np.array([[-1, -1], [1, -1],
                                  [1, 1], [-1, 1]])
        rotated_points = [(cos_a * x - sin_a * y, sin_a * x + cos_a * y) for x, y in square_points]
        screen_points = [self.to_screen(np.array(p) + position) for p in rotated_points]

        pygame.draw.polygon(self.screen, (0, 0, 255), screen_points)  

        front_endpoint = position + np.array([cos_a, sin_a]) * 1.5
        pygame.draw.line(self.screen, (0, 0, 0), self.to_screen(position), self.to_screen(front_endpoint), 2)  

        velocity_endpoint = position + velocity * 0.5  
        pygame.draw.line(self.screen, (255, 0, 0), self.to_screen(position), self.to_screen(velocity_endpoint), 3)  

        for pos, acc in zip(motor_positions, motor_accelerations):
            pygame.draw.circle(self.screen, (200, 150, 0), self.to_screen(pos), 5)  
            acc_endpoint = pos + acc * 0.5  
            pygame.draw.line(self.screen, (200, 150, 0), self.to_screen(pos), self.to_screen(acc_endpoint), 2)  
        text_surface = self.font.render(debug_text, False, (0, 0, 0))
        self.screen.blit(text_surface, (0,0))

        raw_frame = pygame.image.tostring(self.screen, "RGB")
        self.ffmpeg_proc.stdin.write(raw_frame)
        

        pygame.display.flip()
        self.clock.tick(self.fps)

    def render(self):
        self.ffmpeg_proc.stdin.close()
        self.ffmpeg_proc.wait()

        pygame.quit()
        print(f"Video saved at: {os.path.join(self.output_dir, 'animation.mp4')}")