"""Action Sequence Panel"""

import asyncio
import json
import sys
import os
from typing import Optional, TYPE_CHECKING
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QPushButton, QLabel, QSpinBox, QComboBox,
    QTableWidget, QTableWidgetItem, QHeaderView,
    QTextEdit, QFileDialog, QMessageBox
)
from PySide6.QtCore import Qt

# Import from parent's common_imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common_imports import sdk, logger, libstark

from .i18n import tr

if TYPE_CHECKING:
    from .shared_data import SharedDataManager


def run_async(coro):
    """Run async coroutine in a new event loop (for Qt callbacks)"""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        return loop.run_until_complete(coro)
    finally:
        loop.close()


# Preset action sequences
PRESET_SEQUENCES = {
    "fist": {
        "name_en": "Fist",
        "name_zh": "握拳",
        "actions": [
            {"index": 0, "duration_ms": 1000, "mode": 1,
             "positions": [45, 44, 84, 84, 84, 84],
             "durations": [1000, 1000, 1000, 1000, 1000, 1000],
             "speeds": [0, 0, 0, 0, 0, 0], "currents": [0, 0, 0, 0, 0, 0]},
        ]
    },
    "open": {
        "name_en": "Open",
        "name_zh": "张开",
        "actions": [
            {"index": 0, "duration_ms": 1000, "mode": 1,
             "positions": [0, 0, 0, 0, 0, 0],
             "durations": [1000, 1000, 1000, 1000, 1000, 1000],
             "speeds": [0, 0, 0, 0, 0, 0], "currents": [0, 0, 0, 0, 0, 0]},
        ]
    },
    "pinch": {
        "name_en": "Pinch",
        "name_zh": "捏取",
        "actions": [
            {"index": 0, "duration_ms": 1000, "mode": 1,
             "positions": [30, 30, 0, 0, 0, 0],
             "durations": [1000, 1000, 1000, 1000, 1000, 1000],
             "speeds": [0, 0, 0, 0, 0, 0], "currents": [0, 0, 0, 0, 0, 0]},
        ]
    },
    "point": {
        "name_en": "Point",
        "name_zh": "指向",
        "actions": [
            {"index": 0, "duration_ms": 1000, "mode": 1,
             "positions": [45, 0, 84, 84, 84, 84],
             "durations": [1000, 1000, 1000, 1000, 1000, 1000],
             "speeds": [0, 0, 0, 0, 0, 0], "currents": [0, 0, 0, 0, 0, 0]},
        ]
    },
    "wave": {
        "name_en": "Wave",
        "name_zh": "波浪",
        "actions": [
            {"index": 0, "duration_ms": 500, "mode": 1,
             "positions": [0, 0, 0, 0, 0, 0],
             "durations": [500, 500, 500, 500, 500, 500],
             "speeds": [0, 0, 0, 0, 0, 0], "currents": [0, 0, 0, 0, 0, 0]},
            {"index": 1, "duration_ms": 300, "mode": 1,
             "positions": [0, 84, 0, 0, 0, 0],
             "durations": [300, 300, 300, 300, 300, 300],
             "speeds": [0, 0, 0, 0, 0, 0], "currents": [0, 0, 0, 0, 0, 0]},
            {"index": 2, "duration_ms": 300, "mode": 1,
             "positions": [0, 0, 84, 0, 0, 0],
             "durations": [300, 300, 300, 300, 300, 300],
             "speeds": [0, 0, 0, 0, 0, 0], "currents": [0, 0, 0, 0, 0, 0]},
            {"index": 3, "duration_ms": 300, "mode": 1,
             "positions": [0, 0, 0, 84, 0, 0],
             "durations": [300, 300, 300, 300, 300, 300],
             "speeds": [0, 0, 0, 0, 0, 0], "currents": [0, 0, 0, 0, 0, 0]},
            {"index": 4, "duration_ms": 300, "mode": 1,
             "positions": [0, 0, 0, 0, 84, 0],
             "durations": [300, 300, 300, 300, 300, 300],
             "speeds": [0, 0, 0, 0, 0, 0], "currents": [0, 0, 0, 0, 0, 0]},
        ]
    },
}


class ActionSequencePanel(QWidget):
    """Action Sequence Panel

    Uses SharedDataManager for device state.
    """

    def __init__(self):
        super().__init__()
        self.shared_data: Optional['SharedDataManager'] = None
        self.current_actions = []

        self._setup_ui()
        self.update_texts()

    # Properties to get device state from shared_data
    @property
    def device(self):
        return self.shared_data.device if self.shared_data else None

    @property
    def slave_id(self):
        return self.shared_data.slave_id if self.shared_data else 1

    @property
    def device_info(self):
        return self.shared_data.device_info if self.shared_data else None

    def _setup_ui(self):
        """Setup UI"""
        layout = QVBoxLayout()
        self.setLayout(layout)

        # Built-in gestures (built into device firmware, can be run directly)
        self.builtin_group = QGroupBox("Built-in Gestures")
        builtin_layout = QHBoxLayout()

        self.builtin_label = QLabel("Gesture:")
        builtin_layout.addWidget(self.builtin_label)

        self.builtin_combo = QComboBox()
        self.builtin_combo.addItem("Open (张开)", "DefaultGestureOpen")
        self.builtin_combo.addItem("Fist (握拳)", "DefaultGestureFist")
        self.builtin_combo.addItem("Pinch Two (两指捏)", "DefaultGesturePinchTwo")
        self.builtin_combo.addItem("Pinch Three (三指捏)", "DefaultGesturePinchThree")
        self.builtin_combo.addItem("Pinch Side (侧捏)", "DefaultGesturePinchSide")
        self.builtin_combo.addItem("Point (指向)", "DefaultGesturePoint")
        builtin_layout.addWidget(self.builtin_combo)

        self.run_builtin_btn = QPushButton("▶ Run")
        self.run_builtin_btn.clicked.connect(self._run_builtin_gesture)
        builtin_layout.addWidget(self.run_builtin_btn)

        builtin_layout.addStretch()
        self.builtin_group.setLayout(builtin_layout)
        layout.addWidget(self.builtin_group)

        # Preset action templates (load into editor)
        self.preset_group = QGroupBox()
        preset_layout = QHBoxLayout()

        self.preset_label = QLabel()
        preset_layout.addWidget(self.preset_label)

        self.preset_combo = QComboBox()
        preset_layout.addWidget(self.preset_combo)

        self.load_preset_btn = QPushButton()
        self.load_preset_btn.clicked.connect(self._load_preset)
        preset_layout.addWidget(self.load_preset_btn)

        preset_layout.addStretch()
        self.preset_group.setLayout(preset_layout)
        layout.addWidget(self.preset_group)

        # Action list
        self.actions_group = QGroupBox()
        actions_layout = QVBoxLayout()

        self.actions_table = QTableWidget()
        self.actions_table.setColumnCount(5)
        self.actions_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        actions_layout.addWidget(self.actions_table)

        # Action edit buttons
        edit_layout = QHBoxLayout()

        self.add_action_btn = QPushButton()
        self.add_action_btn.clicked.connect(self._add_action)
        edit_layout.addWidget(self.add_action_btn)

        self.remove_action_btn = QPushButton()
        self.remove_action_btn.clicked.connect(self._remove_action)
        edit_layout.addWidget(self.remove_action_btn)

        self.clear_actions_btn = QPushButton()
        self.clear_actions_btn.clicked.connect(self._clear_actions)
        edit_layout.addWidget(self.clear_actions_btn)

        edit_layout.addStretch()
        actions_layout.addLayout(edit_layout)

        self.actions_group.setLayout(actions_layout)
        layout.addWidget(self.actions_group)

        # Execute control
        self.execute_group = QGroupBox()
        execute_layout = QHBoxLayout()

        self.slot_label = QLabel()
        execute_layout.addWidget(self.slot_label)

        self.slot_spin = QSpinBox()
        self.slot_spin.setRange(1, 6)
        self.slot_spin.setValue(1)
        execute_layout.addWidget(self.slot_spin)

        self.upload_btn = QPushButton()
        self.upload_btn.clicked.connect(self._upload_sequence)
        execute_layout.addWidget(self.upload_btn)

        self.run_btn = QPushButton()
        self.run_btn.clicked.connect(self._run_sequence)
        execute_layout.addWidget(self.run_btn)

        self.stop_btn = QPushButton()
        self.stop_btn.clicked.connect(self._stop_sequence)
        execute_layout.addWidget(self.stop_btn)

        execute_layout.addStretch()
        self.execute_group.setLayout(execute_layout)
        layout.addWidget(self.execute_group)

        # File operations
        self.file_group = QGroupBox()
        file_layout = QHBoxLayout()

        self.import_btn = QPushButton()
        self.import_btn.clicked.connect(self._import_sequence)
        file_layout.addWidget(self.import_btn)

        self.export_btn = QPushButton()
        self.export_btn.clicked.connect(self._export_sequence)
        file_layout.addWidget(self.export_btn)

        file_layout.addStretch()
        self.file_group.setLayout(file_layout)
        layout.addWidget(self.file_group)

        layout.addStretch()


    def update_texts(self):
        """Update texts"""
        from .i18n import get_i18n
        lang = get_i18n().current_language

        self.preset_group.setTitle(tr("preset_actions"))
        self.preset_label.setText(tr("preset") + ":")
        self.load_preset_btn.setText(tr("btn_load"))

        # Update preset dropdown
        self.preset_combo.clear()
        for key, preset in PRESET_SEQUENCES.items():
            name = preset["name_zh"] if lang == "zh" else preset["name_en"]
            self.preset_combo.addItem(name, key)

        self.actions_group.setTitle(tr("action_list"))
        self.actions_table.setHorizontalHeaderLabels([
            tr("action_index"),
            tr("action_duration"),
            tr("action_mode"),
            tr("action_positions"),
            tr("action_durations")
        ])

        self.add_action_btn.setText(tr("btn_add"))
        self.remove_action_btn.setText(tr("btn_remove"))
        self.clear_actions_btn.setText(tr("btn_clear"))

        self.execute_group.setTitle(tr("execute_control"))
        self.slot_label.setText(tr("custom_slot") + ":")
        self.upload_btn.setText(tr("btn_upload"))
        self.run_btn.setText(tr("btn_run"))
        self.stop_btn.setText(tr("btn_stop"))

        self.file_group.setTitle(tr("file_operations"))
        self.import_btn.setText(tr("btn_import"))
        self.export_btn.setText(tr("btn_export"))

    def set_device(self, device, slave_id, device_info, shared_data=None):
        """Set device"""
        self.shared_data = shared_data

    def _run_builtin_gesture(self):
        """Run built-in gesture"""
        if not self.device:
            return
        run_async(self._async_run_builtin())

    async def _async_run_builtin(self):
        """Async run built-in gesture"""
        if not self.device:
            return
        try:
            gesture_name = self.builtin_combo.currentData()
            action_id = getattr(libstark.ActionSequenceId, gesture_name)
            await self.device.run_action_sequence(self.slave_id, action_id)
        except Exception as e:
            QMessageBox.warning(self, tr("error"), f"Run gesture failed: {e}")

    def _load_preset(self):
        """Load preset"""
        key = self.preset_combo.currentData()
        if key and key in PRESET_SEQUENCES:
            self.current_actions = PRESET_SEQUENCES[key]["actions"].copy()
            self._update_table()

    def _update_table(self):
        """Update table"""
        self.actions_table.setRowCount(len(self.current_actions))
        for i, action in enumerate(self.current_actions):
            self.actions_table.setItem(i, 0, QTableWidgetItem(str(action["index"])))
            self.actions_table.setItem(i, 1, QTableWidgetItem(str(action["duration_ms"])))
            self.actions_table.setItem(i, 2, QTableWidgetItem(str(action["mode"])))
            self.actions_table.setItem(i, 3, QTableWidgetItem(str(action["positions"])))
            self.actions_table.setItem(i, 4, QTableWidgetItem(str(action["durations"])))

    def _add_action(self):
        """Add action"""
        new_action = {
            "index": len(self.current_actions),
            "duration_ms": 1000,
            "mode": 1,
            "positions": [0, 0, 0, 0, 0, 0],
            "durations": [1000, 1000, 1000, 1000, 1000, 1000],
            "speeds": [0, 0, 0, 0, 0, 0],
            "currents": [0, 0, 0, 0, 0, 0]
        }
        self.current_actions.append(new_action)
        self._update_table()

    def _remove_action(self):
        """Remove action"""
        row = self.actions_table.currentRow()
        if row >= 0 and row < len(self.current_actions):
            self.current_actions.pop(row)
            # Re-index
            for i, action in enumerate(self.current_actions):
                action["index"] = i
            self._update_table()

    def _clear_actions(self):
        """Clear actions"""
        self.current_actions = []
        self._update_table()

    def _action_to_list(self, action):
        """Convert action to list format"""
        return (
            [action["index"], action["duration_ms"], action["mode"]]
            + action["positions"]
            + action["durations"]
            + action["speeds"]
            + action["currents"]
        )

    def _upload_sequence(self):
        """Upload action sequence"""
        if not self.device or not self.current_actions:
            return
        run_async(self._async_upload())

    async def _async_upload(self):
        """Async upload"""
        if not self.device:
            return
        try:
            slot = self.slot_spin.value()
            # Get corresponding ActionSequenceId
            action_id = getattr(libstark.ActionSequenceId, f"CustomGesture{slot}")

            sequences = [self._action_to_list(a) for a in self.current_actions]
            await self.device.transfer_action_sequence(self.slave_id, action_id, sequences)

            QMessageBox.information(self, tr("success"), tr("upload_success"))
        except Exception as e:
            QMessageBox.warning(self, tr("error"), f"{tr('upload_failed')}: {e}")

    def _run_sequence(self):
        """Run action sequence"""
        if not self.device:
            return
        run_async(self._async_run())

    async def _async_run(self):
        """Async run"""
        if not self.device:
            return
        try:
            slot = self.slot_spin.value()
            action_id = getattr(libstark.ActionSequenceId, f"CustomGesture{slot}")
            await self.device.run_action_sequence(self.slave_id, action_id)
        except Exception as e:
            QMessageBox.warning(self, tr("error"), f"{tr('run_failed')}: {e}")

    def _stop_sequence(self):
        """Stop action sequence"""
        if not self.device:
            return
        run_async(self._async_stop())

    async def _async_stop(self):
        """Async stop"""
        if not self.device:
            return
        try:
            for i in range(6):
                await self.device.stop_motor(self.slave_id, i)
        except Exception as e:
            print(f"Stop failed: {e}")

    def _import_sequence(self):
        """Import action sequence"""
        filename, _ = QFileDialog.getOpenFileName(
            self, tr("btn_import"), "", "JSON Files (*.json)"
        )
        if filename:
            try:
                with open(filename, 'r') as f:
                    data = json.load(f)
                    self.current_actions = data.get("actions", [])
                    self._update_table()
            except Exception as e:
                QMessageBox.warning(self, tr("error"), f"{tr('import_failed')}: {e}")

    def _export_sequence(self):
        """Export action sequence"""
        if not self.current_actions:
            return

        filename, _ = QFileDialog.getSaveFileName(
            self, tr("btn_export"), "", "JSON Files (*.json)"
        )
        if filename:
            try:
                with open(filename, 'w') as f:
                    json.dump({"actions": self.current_actions}, f, indent=2)
            except Exception as e:
                QMessageBox.warning(self, tr("error"), f"{tr('export_failed')}: {e}")
