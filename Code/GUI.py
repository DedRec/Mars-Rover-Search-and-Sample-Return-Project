from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(311, 406)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.lineEdit = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit.setGeometry(QtCore.QRect(30, 220, 91, 20))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.lineEdit.setFont(font)
        self.lineEdit.setAutoFillBackground(False)
        self.lineEdit.setObjectName("lineEdit")
        self.lineEdit_2 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_2.setGeometry(QtCore.QRect(30, 110, 51, 20))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.lineEdit_2.setFont(font)
        self.lineEdit_2.setAutoFillBackground(False)
        self.lineEdit_2.setObjectName("lineEdit_2")
        self.lineEdit_3 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_3.setGeometry(QtCore.QRect(30, 160, 81, 20))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.lineEdit_3.setFont(font)
        self.lineEdit_3.setAutoFillBackground(False)
        self.lineEdit_3.setObjectName("lineEdit_3")
        self.RotatingTimer = QtWidgets.QLineEdit(self.centralwidget)
        self.RotatingTimer.setGeometry(QtCore.QRect(30, 40, 91, 20))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.RotatingTimer.setFont(font)
        self.RotatingTimer.setAutoFillBackground(False)
        self.RotatingTimer.setObjectName("RotatingTimer")
        self.Rotatingtimer = QtWidgets.QProgressBar(self.centralwidget)
        self.Rotatingtimer.setGeometry(QtCore.QRect(140, 40, 141, 23))
        font = QtGui.QFont()
        font.setStyleStrategy(QtGui.QFont.PreferDefault)
        self.Rotatingtimer.setFont(font)
        self.Rotatingtimer.setMaximum(30)
        self.Rotatingtimer.setProperty("value", 15)
        self.Rotatingtimer.setTextVisible(False)
        self.Rotatingtimer.setOrientation(QtCore.Qt.Horizontal)
        self.Rotatingtimer.setInvertedAppearance(False)
        self.Rotatingtimer.setTextDirection(QtWidgets.QProgressBar.BottomToTop)
        self.Rotatingtimer.setObjectName("Rotatingtimer")
        self.steeringcount = QtWidgets.QProgressBar(self.centralwidget)
        self.steeringcount.setEnabled(True)
        self.steeringcount.setGeometry(QtCore.QRect(140, 220, 141, 23))
        font = QtGui.QFont()
        font.setStyleStrategy(QtGui.QFont.PreferDefault)
        self.steeringcount.setFont(font)
        self.steeringcount.setAutoFillBackground(False)
        self.steeringcount.setInputMethodHints(QtCore.Qt.ImhNone)
        self.steeringcount.setMaximum(30)
        self.steeringcount.setProperty("value", 15)
        self.steeringcount.setTextVisible(False)
        self.steeringcount.setOrientation(QtCore.Qt.Horizontal)
        self.steeringcount.setInvertedAppearance(False)
        self.steeringcount.setTextDirection(QtWidgets.QProgressBar.BottomToTop)
        self.steeringcount.setObjectName("steeringcount")
        self.POScount = QtWidgets.QProgressBar(self.centralwidget)
        self.POScount.setGeometry(QtCore.QRect(140, 270, 141, 23))
        font = QtGui.QFont()
        font.setStyleStrategy(QtGui.QFont.PreferDefault)
        self.POScount.setFont(font)
        self.POScount.setMaximum(30)
        self.POScount.setProperty("value", 15)
        self.POScount.setTextVisible(False)
        self.POScount.setOrientation(QtCore.Qt.Horizontal)
        self.POScount.setInvertedAppearance(False)
        self.POScount.setTextDirection(QtWidgets.QProgressBar.BottomToTop)
        self.POScount.setObjectName("POScount")
        self.lineEdit_7 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_7.setGeometry(QtCore.QRect(30, 270, 71, 20))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.lineEdit_7.setFont(font)
        self.lineEdit_7.setAutoFillBackground(False)
        self.lineEdit_7.setObjectName("lineEdit_7")
        self.Mode = QtWidgets.QLineEdit(self.centralwidget)
        self.Mode.setGeometry(QtCore.QRect(130, 110, 113, 20))
        self.Mode.setObjectName("Mode")
        self.rock_flag = QtWidgets.QLineEdit(self.centralwidget)
        self.rock_flag.setGeometry(QtCore.QRect(140, 160, 91, 20))
        self.rock_flag.setObjectName("rock_flag")
        self.steeringangle = QtWidgets.QProgressBar(self.centralwidget)
        self.steeringangle.setGeometry(QtCore.QRect(130, 340, 141, 21))
        font = QtGui.QFont()
        font.setStyleStrategy(QtGui.QFont.PreferDefault)
        self.steeringangle.setFont(font)
        self.steeringangle.setMaximum(30)
        self.steeringangle.setProperty("value", 15)
        self.steeringangle.setTextVisible(False)
        self.steeringangle.setOrientation(QtCore.Qt.Horizontal)
        self.steeringangle.setInvertedAppearance(False)
        self.steeringangle.setTextDirection(QtWidgets.QProgressBar.BottomToTop)
        self.steeringangle.setObjectName("steeringangle")
        self.lineEdit_8 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_8.setGeometry(QtCore.QRect(20, 340, 91, 20))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.lineEdit_8.setFont(font)
        self.lineEdit_8.setAutoFillBackground(False)
        self.lineEdit_8.setObjectName("lineEdit_8")
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.lineEdit.setText(_translate("MainWindow", "Steering Count"))
        self.lineEdit_2.setText(_translate("MainWindow", "MODE"))
        self.lineEdit_3.setText(_translate("MainWindow", "rock_flag "))
        self.RotatingTimer.setText(_translate("MainWindow", "Rotating timer"))
        self.lineEdit_7.setText(_translate("MainWindow", "POS Count"))
        self.lineEdit_8.setText(_translate("MainWindow", "Steering angle"))

if __name__ == '__main__':
    print("sys")
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
