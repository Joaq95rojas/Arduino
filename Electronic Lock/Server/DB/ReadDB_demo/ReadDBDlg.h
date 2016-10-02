// ReadDBDlg.h : header file
//

#if !defined(AFX_READDBDLG_H__1983DA87_05C7_11D5_A855_EFDD0E6B2278__INCLUDED_)
#define AFX_READDBDLG_H__1983DA87_05C7_11D5_A855_EFDD0E6B2278__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

/////////////////////////////////////////////////////////////////////////////
// CReadDBDlg dialog

class CReadDBDlg : public CDialog
{
// Construction
public:
	CReadDBDlg(CWnd* pParent = NULL);	// standard constructor
     void ResetListControl();

// Dialog Data
	//{{AFX_DATA(CReadDBDlg)
	enum { IDD = IDD_READDB_DIALOG };
	CListCtrl	m_ListControl;
	//}}AFX_DATA

	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CReadDBDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	//{{AFX_MSG(CReadDBDlg)
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	afx_msg void OnRead();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_READDBDLG_H__1983DA87_05C7_11D5_A855_EFDD0E6B2278__INCLUDED_)
