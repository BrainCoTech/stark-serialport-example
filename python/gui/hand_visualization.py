"""2D Hand Visualization Widget"""

import math
from typing import List

from PySide6.QtWidgets import QWidget
from PySide6.QtCore import Qt, QPointF, QRectF
from PySide6.QtGui import QPainter, QPen, QBrush, QColor, QPainterPath, QLinearGradient


class HandVisualization(QWidget):
    """2D hand diagram showing finger positions and touch intensity"""
    
    # Finger configuration: (base_x, base_y, length, base_angle)
    # Positions relative to palm center, angles in degrees (0 = pointing up)
    FINGER_CONFIG = [
        (-60, -20, 50, -30),   # Thumb
        (-30, -80, 60, -10),   # Index
        (0, -85, 65, 0),       # Middle
        (30, -80, 55, 10),     # Ring
        (55, -70, 45, 20),     # Pinky
    ]
    
    FINGER_COLORS = [
        QColor(255, 107, 107),  # Thumb - Red
        QColor(78, 205, 196),   # Index - Teal
        QColor(69, 183, 209),   # Middle - Blue
        QColor(255, 160, 122),  # Ring - Orange
        QColor(152, 216, 200),  # Pinky - Green
    ]
    
    def __init__(self):
        super().__init__()
        self.finger_positions = [0] * 5  # 0-1000 (0=open, 1000=closed) - visual fingers only
        self.touch_intensities = [0] * 5  # 0-1000
        
        self.setMinimumSize(200, 250)
    
    def set_finger_positions(self, positions: List[int]):
        """Update finger positions (0-1000)
        
        Motor has 6 DOF: [Thumb, ThumbAux, Index, Middle, Ring, Pinky]
        Visual has 5 fingers: [Thumb, Index, Middle, Ring, Pinky]
        We skip ThumbAux (index 1) for visualization.
        """
        # Map 6 motor positions to 5 visual fingers (skip ThumbAux at index 1)
        if len(positions) >= 6:
            visual_positions = [positions[0], positions[2], positions[3], positions[4], positions[5]]
        else:
            visual_positions = positions[:5]
        
        for i in range(min(5, len(visual_positions))):
            self.finger_positions[i] = max(0, min(1000, visual_positions[i]))
        self.update()
    
    def set_touch_intensities(self, intensities: List[int]):
        """Update touch intensity colors (0-1000)"""
        for i in range(min(5, len(intensities))):
            self.touch_intensities[i] = max(0, min(1000, intensities[i]))
        self.update()
    
    def _position_to_angle(self, position: int, finger_idx: int) -> float:
        """Convert position (0-1000) to finger bend angle in degrees"""
        # 0 = straight, 1000 = fully bent (90 degrees)
        max_bend = 90 if finger_idx > 0 else 60  # Thumb bends less
        return (position / 1000.0) * max_bend
    
    def _intensity_to_color(self, intensity: int) -> QColor:
        """Convert intensity (0-1000) to color gradient (blue -> green -> yellow -> red)"""
        t = intensity / 1000.0
        
        if t < 0.33:
            # Blue to Green
            r = int(0 + (0 - 0) * (t / 0.33))
            g = int(100 + (200 - 100) * (t / 0.33))
            b = int(255 + (100 - 255) * (t / 0.33))
        elif t < 0.66:
            # Green to Yellow
            tt = (t - 0.33) / 0.33
            r = int(0 + (255 - 0) * tt)
            g = int(200 + (200 - 200) * tt)
            b = int(100 + (0 - 100) * tt)
        else:
            # Yellow to Red
            tt = (t - 0.66) / 0.34
            r = int(255)
            g = int(200 + (50 - 200) * tt)
            b = int(0)
        
        return QColor(r, g, b)
    
    def paintEvent(self, event):
        """Custom painting of hand diagram"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Calculate center and scale
        w = self.width()
        h = self.height()
        center_x = w / 2
        center_y = h / 2 + 30
        scale = min(w, h) / 300.0
        
        # Draw palm
        self._draw_palm(painter, center_x, center_y, scale)
        
        # Draw fingers
        for i in range(5):
            self._draw_finger(painter, center_x, center_y, scale, i)
    
    def _draw_palm(self, painter: QPainter, cx: float, cy: float, scale: float):
        """Draw the palm shape"""
        painter.setPen(QPen(QColor(100, 100, 100), 2))
        painter.setBrush(QBrush(QColor(255, 220, 185)))
        
        # Palm as rounded rectangle
        palm_w = 100 * scale
        palm_h = 80 * scale
        rect = QRectF(cx - palm_w/2, cy - palm_h/2, palm_w, palm_h)
        painter.drawRoundedRect(rect, 15 * scale, 15 * scale)
    
    def _draw_finger(self, painter: QPainter, cx: float, cy: float, scale: float, finger_idx: int):
        """Draw a single finger with current position and touch intensity"""
        config = self.FINGER_CONFIG[finger_idx]
        base_x = cx + config[0] * scale
        base_y = cy + config[1] * scale
        length = config[2] * scale
        base_angle = config[3]
        
        # Calculate bend angle from position
        bend_angle = self._position_to_angle(self.finger_positions[finger_idx], finger_idx)
        
        # Finger has 2 segments
        seg1_len = length * 0.6
        seg2_len = length * 0.4
        
        # First segment angle (from base)
        angle1_rad = math.radians(base_angle - 90 + bend_angle * 0.3)
        seg1_end_x = base_x + seg1_len * math.cos(angle1_rad)
        seg1_end_y = base_y + seg1_len * math.sin(angle1_rad)
        
        # Second segment angle (more bend)
        angle2_rad = angle1_rad + math.radians(bend_angle * 0.7)
        seg2_end_x = seg1_end_x + seg2_len * math.cos(angle2_rad)
        seg2_end_y = seg1_end_y + seg2_len * math.sin(angle2_rad)
        
        # Draw finger segments
        finger_width = 12 * scale if finger_idx == 0 else 10 * scale
        painter.setPen(QPen(QColor(100, 100, 100), 1))
        painter.setBrush(QBrush(QColor(255, 220, 185)))
        
        # Draw as thick lines (simplified)
        pen = QPen(QColor(255, 220, 185), finger_width, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin)
        painter.setPen(pen)
        painter.drawLine(QPointF(base_x, base_y), QPointF(seg1_end_x, seg1_end_y))
        painter.drawLine(QPointF(seg1_end_x, seg1_end_y), QPointF(seg2_end_x, seg2_end_y))
        
        # Draw outline
        pen = QPen(QColor(100, 100, 100), 1)
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)
        
        # Draw fingertip (neutral color, no touch intensity)
        tip_radius = 8 * scale if finger_idx == 0 else 6 * scale
        painter.setPen(QPen(QColor(80, 80, 80), 1))
        painter.setBrush(QBrush(QColor(255, 200, 170)))  # Skin color
        painter.drawEllipse(QPointF(seg2_end_x, seg2_end_y), tip_radius, tip_radius)
