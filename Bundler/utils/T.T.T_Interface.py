# -*- coding: utf-8 -*-
from PyQt4.QtGui import *
from PyQt4.QtCore import *

import sys
import os
import threading
import subprocess
import time

from PyQt4 import QtOpenGL

QTextCodec.setCodecForTr(QTextCodec.codecForName("utf8"))


class InterfaceDialog(QDialog):
    def __init__(self, parent=None):
        super(InterfaceDialog, self).__init__(parent)
        self.setWindowTitle(self.tr("T.T.T"))

        self.local_path = QLineEdit()

	self.textTest = QLineEdit()
	self.textTest.setReadOnly(True)
	self.textTest.setMaxLength(32)

        self.choose_button = QPushButton(self.tr("选择"))
        self.action_button = QPushButton(self.tr("执行"))

        top_layout = QHBoxLayout()
        top_layout.addWidget(self.local_path)
        top_layout.addWidget(self.choose_button)
        top_layout.addWidget(self.action_button)
	#top_layout.addWidget(self.textTest)

        # # about progress
        medium_layout = QHBoxLayout()
        # progressBar = QProgressBar()
        # medium_layout.addWidget(progressBar)
        # # if (shell_threading.is_alive):

        self.end_button = QPushButton(self.tr("结束"))

        bottom_layout = QHBoxLayout()
        bottom_layout.addWidget(self.end_button)

        mainLayout = QGridLayout(self)
        mainLayout.setMargin(15)
        mainLayout.setMargin(10)
        mainLayout.addLayout(top_layout, 0, 0)
        mainLayout.addLayout(medium_layout, 1, 0, 1, 1)
        mainLayout.addLayout(bottom_layout, 2, 0, 1, 1)
        #mainLayout.addLayout(QLayout.SetFixedSize)

        self.choose_button.clicked.connect(self.choose_button_click)
        self.action_button.clicked.connect(self.action_button_click)
        self.end_button.clicked.connect(self.end_button_click)
	
	#self.pbar=QProgressBar(self)
	medium_layout.addWidget(self.textTest)
        
	self.lastOpertion=""

    def choose_button_click(self):
        absolute_path = QFileDialog.getOpenFileName(self, 'Open file', '.', "mp4 files (*.mp4)")
        self.local_path.setText(absolute_path)

    def parse_string(self,inStr):
	temp=inStr.replace(' ','')
	output=self.lastOpertion
	if(temp[:4]=="Usin"):
	    output="Using config file"
	elif(temp[:4]=="[Ext"):
	    output="Extracting exif tags from image"
	elif(temp[:4]=="[-Ex"):
	    output="Extracting keypoints"
	elif(temp[:4]=="[Key"):
	    output="Key Match Full"
	elif(temp[:4]=="[-Ru"):
	    output="Running Bundler"
	elif(temp[:4]=="[-Do"):
	    output="Done"
	elif(temp[:4]=="[Rea"):
	    output="Read Bundle File"
	elif(temp[:4]=="Undi"):
	    output="Undistorting image"
	elif(temp[:4]=="[Wri"):
	    output="Write Bundle File"
	elif(temp[:4]=="pmvs"):
	    output="Expanding patches"
	self.lastOpertion=output
	return output
	    

    def run_shell(self,proc): 
	for line in iter(proc.stdout.readline, b''):
	    self.textTest.setText(self.parse_string(line))
	    

    def action_button_click(self):
	proc=subprocess.Popen("/home/efrost/Documents/bundler_sfm/RunBundler.sh "+str(self.local_path.text()),
					shell=True,
					stdout=subprocess.PIPE)
	shell_threading = threading.Thread(target=self.run_shell, args=(proc, ))
	shell_threading.start()

    def end_button_click(self):
        self.close()


app = QApplication(sys.argv)
dialog = InterfaceDialog()
dialog.show()
app.exec_()
