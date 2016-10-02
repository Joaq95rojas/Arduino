; CLW file contains information for the MFC ClassWizard

[General Info]
Version=1
LastClass=CReadDBDlg
LastTemplate=CDialog
NewFileInclude1=#include "stdafx.h"
NewFileInclude2=#include "ReadDB.h"

ClassCount=3
Class1=CReadDBApp
Class2=CReadDBDlg
Class3=CAboutDlg

ResourceCount=5
Resource1=IDD_ABOUTBOX
Resource2=IDR_MAINFRAME
Resource3=IDD_READDB_DIALOG
Resource4=IDD_ABOUTBOX (English (U.S.))
Resource5=IDD_READDB_DIALOG (English (U.S.))

[CLS:CReadDBApp]
Type=0
HeaderFile=ReadDB.h
ImplementationFile=ReadDB.cpp
Filter=N

[CLS:CReadDBDlg]
Type=0
HeaderFile=ReadDBDlg.h
ImplementationFile=ReadDBDlg.cpp
Filter=D
BaseClass=CDialog
VirtualFilter=dWC
LastObject=IDC_READ

[CLS:CAboutDlg]
Type=0
HeaderFile=ReadDBDlg.h
ImplementationFile=ReadDBDlg.cpp
Filter=D

[DLG:IDD_ABOUTBOX]
Type=1
ControlCount=4
Control1=IDC_STATIC,static,1342177283
Control2=IDC_STATIC,static,1342308352
Control3=IDC_STATIC,static,1342308352
Control4=IDOK,button,1342373889
Class=CAboutDlg


[DLG:IDD_READDB_DIALOG]
Type=1
ControlCount=3
Control1=IDOK,button,1342242817
Control2=IDCANCEL,button,1342242816
Control3=IDC_STATIC,static,1342308352
Class=CReadDBDlg

[DLG:IDD_READDB_DIALOG (English (U.S.))]
Type=1
Class=CReadDBDlg
ControlCount=5
Control1=IDOK,button,1342242817
Control2=IDCANCEL,button,1342242816
Control3=IDC_READ,button,1342242816
Control4=IDC_ListControl,SysListView32,1350565901
Control5=IDC_STATIC,static,1342177281

[DLG:IDD_ABOUTBOX (English (U.S.))]
Type=1
Class=?
ControlCount=4
Control1=IDC_STATIC,static,1342177283
Control2=IDC_STATIC,static,1342308480
Control3=IDC_STATIC,static,1342308352
Control4=IDOK,button,1342373889

