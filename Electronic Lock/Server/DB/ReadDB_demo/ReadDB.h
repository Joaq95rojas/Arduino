// ReadDB.h : main header file for the READDB application
//

#if !defined(AFX_READDB_H__1983DA85_05C7_11D5_A855_EFDD0E6B2278__INCLUDED_)
#define AFX_READDB_H__1983DA85_05C7_11D5_A855_EFDD0E6B2278__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#ifndef __AFXWIN_H__
	#error include 'stdafx.h' before including this file for PCH
#endif

#include "resource.h"		// main symbols

/////////////////////////////////////////////////////////////////////////////
// CReadDBApp:
// See ReadDB.cpp for the implementation of this class
//

class CReadDBApp : public CWinApp
{
public:
	CReadDBApp();

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CReadDBApp)
	public:
	virtual BOOL InitInstance();
	//}}AFX_VIRTUAL

// Implementation

	//{{AFX_MSG(CReadDBApp)
		// NOTE - the ClassWizard will add and remove member functions here.
		//    DO NOT EDIT what you see in these blocks of generated code !
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};


/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_READDB_H__1983DA85_05C7_11D5_A855_EFDD0E6B2278__INCLUDED_)
