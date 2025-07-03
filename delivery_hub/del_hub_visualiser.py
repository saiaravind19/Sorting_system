import sys
import argparse
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QHBoxLayout
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QColor, QBrush, QFont
from pymodbus.client import ModbusTcpClient
import time
import logging
logging.basicConfig(level= logging.INFO,format="[%(levelname)s]:[%(name)s]:[%(funcName)s]:%(message)s")


class ModbusMonitorUI(QMainWindow):
    def __init__(self, host: str, port: int, num_slaves: int, num_bins: int):
        super().__init__()
        self.logger = logging.getLogger(f"del_hub_viz")
        self.host = host
        self.port = port
        self.num_slaves = num_slaves
        self.num_bins = num_bins

        self.poll_interval = 100
        # Connect Modbus client
        self.client = ModbusTcpClient(self.host, port=self.port)
        while not self.client.connect():
            time.sleep(1)
            self.logger.warning(f"Cannot connect to {self.host}:{self.port}, retrying...")

        self.setWindowTitle("Modbus Bin & Package Monitor")
        self.init_ui()

        # Start polling timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_readings)
        self.timer.start(self.poll_interval)

    def init_ui(self):
        central = QWidget()
        layout = QVBoxLayout()

        # Set up coloumn headers of the UI
        headers = [f"Bin {i+1}" for i in range(self.num_bins)] + ["Pkg Count"]
        self.table = QTableWidget(self.num_slaves, len(headers))
        self.table.setHorizontalHeaderLabels(headers)

        # Label rows headers of the UI 
        self.table.setVerticalHeaderLabels([f"Slave {i}" for i in range(self.num_slaves)])
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        layout.addWidget(self.table)

        
        total_layout = QHBoxLayout()
        total_layout.addStretch()
        self.total_label = QLabel("Total Packages: 0")
        font = QFont()
        font.setPointSize(12)
        font.setBold(True)
        self.total_label.setFont(font)
        self.total_label.setAlignment(Qt.AlignCenter)
        total_layout.addWidget(self.total_label)
        total_layout.addStretch()
        layout.addLayout(total_layout)

        central.setLayout(layout)
        self.setCentralWidget(central)
        self.resize(800, 400)

    def update_readings(self):
        # Clear previous items
        self.table.clearContents()
        total_pkg_count = 0

        for row in range(self.num_slaves):
            unit_id = row  # zero-based slave ID per server context

            # Read bin coil statuses using 'slave' keyword for pymodbus v3+
            coil_reg = self.client.read_coils(0, self.num_bins, slave=unit_id)
            bins = coil_reg.bits
                

            # Read package count register
            pack_count_reg = self.client.read_holding_registers(0, 1, slave=unit_id)

            pkg_count = pack_count_reg.registers[0]
            total_pkg_count += pkg_count

            # Populate bin cells
            for col, occupied in enumerate(bins):
                item = QTableWidgetItem("Occupied" if occupied else "Free")
                item.setTextAlignment(Qt.AlignCenter)
                color = QColor(255, 100, 100) if occupied else QColor(100, 255, 100)
                item.setBackground(QBrush(color))
                self.table.setItem(row, col, item)

            # Display package count
            pkg_item = QTableWidgetItem(str(pkg_count))
            pkg_item.setTextAlignment(Qt.AlignCenter)
            self.table.setItem(row, self.num_bins, pkg_item)

        # Update total label
        self.total_label.setText(f"Total Packages: {total_pkg_count}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Delivery Hub Visualiser using Modbus TCP.")
    parser.add_argument("--host", type=str, default="localhost", help="Modbus server host")
    parser.add_argument("--port", type=int, default=5020, help="Modbus server port")
    parser.add_argument("--no_of_slaves", type=int, default=4, help="Number of Modbus slave ")
    parser.add_argument("--no_of_bins", type=int, default=6, help="No of bins per Delivery Hub")
    args = parser.parse_args()

    app = QApplication(sys.argv)
    window = ModbusMonitorUI(args.host, args.port, args.no_of_slaves, args.no_of_bins)
    window.show()
    sys.exit(app.exec_())
