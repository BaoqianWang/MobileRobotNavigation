#!/usr/bin/env python
'''
Author: David Pierce Walker-Howeell<piercedhowell@gmail.com>
Date Created: 05/26/2020
Description: This python module is a generic pid tuner widget with sliders and
            scale selectors for PID tunning.
'''
import rospy
import rospkg
import os
from python_qt_binding import loadUi, QtGui, QtCore
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QSizePolicy, QApplication

class PID_Tuner_Generic_Widget(QWidget):
    '''
    A Generic widget with sliders and scale selector for PID tunning.
    '''

    def __init__(self):
        '''
        Initialize the Widget

        Parameters:
            N/A
        Returns:
            N/A
        '''
        super(PID_Tuner_Generic_Widget, self).__init__()

        #Load the ui template
        ui_file = os.path.join(rospkg.RosPack().get_path('smile_mobile_gui'), 'resource', 'pid_tuner_generic.ui')
        loadUi(ui_file, self)

        #Initialize variables
        self.k_p = 0.0
        self.k_i = 0.0
        self.k_d = 0.0

        #Connect all of the signals and slots of the sliders
        self._init_sliders()

        #Initialize the combobox's for scaling factor
        self._init_scale_comboboxs()



    def _init_scale_comboboxs(self):
        '''
        Initialize the combobox's with their respective values.

        Parameters:
            N/A
        Returns:
            N/A
        '''
        comboboxs = [self.k_p_scale_combobox, self.k_i_scale_combobox, self.k_d_scale_combobox]

        for combobox in comboboxs:
            combobox.addItems(["0.1", "1.0", "10.0", "100.0", "1000.0"])
            combobox.currentIndexChanged.connect(self._update_scales)

        self._update_scales()

    def _init_sliders(self):
        '''
        Initialize the sliders with the correct ranges.

        Parameters:
            N/A
        Returns:
            N/As
        '''
        sliders = [self.k_p_slider, self.k_i_slider, self.k_d_slider]

        for slider in sliders:
            slider.valueChanged.connect(self._update_gains)
            slider.setMaximum(100)
            slider.setMinimum(0)
            slider.setValue(0)



    def _update_gains(self):
        '''
        Callback Update the gains shown and saved when the slider values change.

        Parameter:
            N/A
        Returns:
            N/A
        '''
        self.k_p = (self.k_p_slider.value() / 100.0) * self.k_p_scale
        self.k_i = (self.k_i_slider.value() / 100.0) * self.k_i_scale
        self.k_d = (self.k_d_slider.value() / 100.0) * self.k_d_scale

        #Update the line edits to reflect the new value
        self.k_p_line_edit.setText(str(float(self.k_p)))
        self.k_i_line_edit.setText(str(float(self.k_i)))
        self.k_d_line_edit.setText(str(float(self.k_d)))

    def _update_scales(self):
        '''
        Callback to update the scale of the sliders

        Parameters:
            N/A
        Returns:
            N/A
        '''
        self.k_p_scale = float(self.k_p_scale_combobox.currentText())
        self.k_i_scale = float(self.k_i_scale_combobox.currentText())
        self.k_d_scale = float(self.k_d_scale_combobox.currentText())
        self._update_gains()

if __name__ == "__main__":
    import sys
    app = QApplication([])
    pid_tuner_generic = PID_Tuner_Generic_Widget()
    pid_tuner_generic.show()
    sys.exit(app.exec_())
