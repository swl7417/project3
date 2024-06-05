#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import String, Int32
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton
from PyQt5.QtCore import QThread, pyqtSignal
import os

class BarcodeScannerApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Barcode Scanner")
        self.layout = QVBoxLayout()
        
        self.label = QLabel("Scan your barcode")
        self.layout.addWidget(self.label)
        
        self.input_barcode = QLineEdit()
        self.input_barcode.returnPressed.connect(self.process_barcode)
        self.layout.addWidget(self.input_barcode)
        
        self.result_label = QLabel()
        self.layout.addWidget(self.result_label)
        
        self.scanned_data_label = QLabel("Scanned Barcode and Destination:")
        self.layout.addWidget(self.scanned_data_label)
        
        self.scanned_data = QLabel()
        self.layout.addWidget(self.scanned_data)
        
        self.setLayout(self.layout)

        self.input_barcode.setFocus()

        self.barcode_dict = {"038000265013": "A", 
                             "9555192507529": "B",   
                             "8806011616153": "B",
                             "8991102308892": "A",
                             "8806358514433": "A",
                             "8804973131578": "B",
                             }
        
        self.scanned_items = []
        self.value_loading_done = {}  # Dictionary to track loading status for each value
        self.setLayout(self.layout)

        # Define ROS publishers
        self.data_pub = rospy.Publisher("data_input", String, queue_size=10)
        self.value_pub = rospy.Publisher("value_input", String, queue_size=10)
        rospy.Subscriber("stop", Int32, self.restart_program)
        rospy.Subscriber("barcode_data", String, self.barcode_callback)
        

        # Add restart button
        self.restart_button = QPushButton("Restart")
        self.restart_button.clicked.connect(self.restart_program)
        self.layout.addWidget(self.restart_button)

    def process_barcode(self):
        input_barcode_text = self.input_barcode.text()
        
        if input_barcode_text in self.barcode_dict:
            if self.barcode_dict[input_barcode_text] != "AlreadyScanned":
                scanned_data = self.barcode_dict[input_barcode_text]
                self.result_label.setText("Destination : " + str(scanned_data))
                self.scanned_items.append((input_barcode_text, str(scanned_data)))
                
                #publish scanned_data
                self.data_pub.publish(self.barcode_dict[input_barcode_text])
                # Update loading status for the scanned value
                self.update_loading_status(scanned_data)

                self.update_scanned_data_display()
                self.barcode_dict[input_barcode_text] = 'AlreadyScanned'
            else:
                self.result_label.setText("Barcode already scanned")
        else:
            self.result_label.setText("Invalid Barcode")
        
        # Clear the text box
        self.input_barcode.clear()

    def process_barcode_with_data(self, input_barcode_text):
        matched_key = self.find_best_match(input_barcode_text)

        if matched_key and self.barcode_dict[matched_key] != "AlreadyScanned":
            scanned_data = self.barcode_dict[matched_key]
            self.result_label.setText("Destination : " + str(scanned_data))
            self.scanned_items.append((input_barcode_text, str(scanned_data)))
            
            # Publish scanned_data
            self.data_pub.publish(scanned_data)
            # Update loading status for the scanned value
            self.update_loading_status(scanned_data)

            self.update_scanned_data_display()
            self.barcode_dict[matched_key] = 'AlreadyScanned'
        else:
            self.result_label.setText("Invalid or already scanned barcode")
        
        # Clear the text box
        self.input_barcode.clear()


    def find_best_match(self, input_barcode_text):
        best_match = None
        max_matches = -1

        for key in self.barcode_dict:
            matches = sum(1 for i in range(min(len(key), len(input_barcode_text))) if key[i] == input_barcode_text[i])
            if matches > max_matches:
                best_match = key
                max_matches = matches

        return best_match

    def restart_program(self, _):
        rospy.signal_shutdown("Restarting")
        python = sys.executable
        os.execl(python, python, *sys.argv)
        
    def update_loading_status(self, value):

        # Check if all occurrences of a specific value have been scanned
        if value not in self.value_loading_done:
            self.value_loading_done[value] = set()
        
        self.value_loading_done[value].add(value)
        if len(self.value_loading_done[value]) == sum(1 for v in self.barcode_dict.values() if v == value):
            print(f"{value} loading done")
            self.value_pub.publish(value)
        
    def update_scanned_data_display(self):
        display_text = ""
        for barcode, data in self.scanned_items:
            display_text += f"Barcode: {barcode}, Destination: {data}\n"
        self.scanned_data.setText(display_text)

    def barcode_callback(self, data):
        # global barcode
        barcode = ""
        try:
            barcode = data.data
            print("barcode: ", barcode)
            if not barcode == "None":
                self.process_barcode_with_data(barcode)
        except ValueError:
            print("Invalid data received, data value must be a string.")


def main():
    # Initialize ROS node
    rospy.init_node('barcode_scanner_node', anonymous=True)
    
    # Create Qt application
    app = QApplication(sys.argv)
    window = BarcodeScannerApp()
    window.show()
    
    # Execute Qt application
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
