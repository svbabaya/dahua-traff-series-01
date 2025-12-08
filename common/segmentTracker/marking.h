#ifndef _CMARKING_H_
#define _CMARKING_H_

#include <algorithm>
#include <vector>

#define maxAB(a,b)            (((a) > (b)) ? (a) : (b))
#define minAB(a,b)            (((a) < (b)) ? (a) : (b))

class CSegParamsRect {
public:
	int l, r, t, b;
	int num;

	inline void AddSeg(const CSegParamsRect &oSP) {
		l = minAB(l, oSP.l);
		r = maxAB(r, oSP.r);
		t = minAB(t, oSP.t);
		b = maxAB(b, oSP.b);
		num += oSP.num;
	}

	void AddPoint(int x, int y) {
		l = minAB(l, x);
		r = maxAB(r, x);
		t = minAB(t, y);
		b = maxAB(b, y);
		++num;
	}

	CSegParamsRect(int x, int y)
		: l(x), r(x), t(y), b(y), num(1) {}
};

inline int FndIdx(const std::vector<int> &oG, int i) {
	while (oG[i] != i) {
		i = oG[i];
	}
	return i;
}

struct C8Conn {
	template <class CSegParams>
	void operator()(const int &x, const int *pPrevLab, const int &iSzxm1,
					int &iTopLab, std::vector<int> &oG, std::vector<CSegParams> &oNewSgs) const {
		int iTopLeftLab(-1);
		if (x>0) {
			iTopLeftLab = pPrevLab[x - 1];
		}

		int iTopRightLab(-1);
		if (x<iSzxm1) {
			iTopRightLab = pPrevLab[x + 1];
		}

		if ((iTopLab == -1) && (iTopLeftLab != -1) && (iTopRightLab != -1)) {
			int iTLIdx = FndIdx(oG, iTopLeftLab);
			int iTRIdx = FndIdx(oG, iTopRightLab);
			if (iTLIdx != iTRIdx) {
				oNewSgs[iTRIdx].AddSeg(oNewSgs[iTLIdx]);
				oG[iTLIdx] = iTRIdx;
			}
		}
		if (iTopLab == -1) {
			if (iTopLeftLab != -1) {
				iTopLab = iTopLeftLab;
			}
			else {
				iTopLab = iTopRightLab;
			}
		}
	}
};

template <class CConn, class  CSegParams>
class CMarkingXY {
	int *pPrevLab;
	int *pCurLab;
	std::vector<int> oG;
	const int iSzxm1;
	std::vector<CSegParams> oNewSgs;
public:
	inline void Swap() {
		std::swap(pCurLab, pPrevLab);
	}

	inline std::vector<CSegParams> GetNewSgs() const {
		std::vector<CSegParams> oR;
		oR.reserve(oNewSgs.size());
		for (int i = 0; i < int(oNewSgs.size()); ++i) {
			if (oG[i] == i) {
				oR.push_back(oNewSgs[i]);
			}
		}
		return oR;
	}

	inline ~CMarkingXY() {
		delete pPrevLab;
		delete pCurLab;
	}

	inline CMarkingXY(int _iW) : iSzxm1(_iW - 1) { /* ,iW(_iW) */
		pPrevLab = new int[_iW];
		pCurLab = new int[_iW];
		for (int ii = 0; ii<_iW; ++ii) {
			pPrevLab[ii] = pCurLab[ii] = -1;
		}
		oG.reserve(1000);
		oNewSgs.reserve(1000);
	}

	inline void Marking(const bool& bBin, int x, int y) {
		int iCurLab(-1);
		if (bBin) {
			int iLeftLab(-1);
			int iLeftIdx(-1);
			if (x > 0) {
				iLeftLab = pCurLab[x - 1];
				if (iLeftLab != -1) {
					iLeftIdx = FndIdx(oG, iLeftLab);
				}
			}
			int iTopLab(pPrevLab[x]);
			int iTopIdx(-1);

			CConn()(x, pPrevLab, iSzxm1, iTopLab, oG, oNewSgs);

			if (iTopLab != -1) {
				iTopIdx = FndIdx(oG, iTopLab);
			}

			if ((iLeftLab == -1) && (iTopLab == -1)) {
				CSegParams oSP(x, y);
				oNewSgs.push_back(oSP);
				const int iL = oNewSgs.size() - 1;
				oG.push_back(iL);
				iCurLab = iL;
			}
			else if ((iLeftLab != -1) && (iTopLab != -1)) {
				oNewSgs[iTopIdx].AddPoint(x, y);
				if (iTopIdx != iLeftIdx) {
					oNewSgs[iTopIdx].AddSeg(oNewSgs[iLeftIdx]);
					oG[iLeftLab] = iTopIdx;
				}
				iCurLab = iTopIdx;
			}
			else if (iLeftLab != -1) {
				oNewSgs[iLeftIdx].AddPoint(x, y);
				iCurLab = iLeftIdx;
			}
			else {
				oNewSgs[iTopIdx].AddPoint(x, y);
				iCurLab = iTopIdx;
			}
		}
		pCurLab[x] = iCurLab;
	}
};

template <class CConn, class  CSegParams = CSegParamsRect>

class CMarking {
	CMarkingXY<CConn, CSegParams> oMXY;
	const int iW;
	int x;
	int y;
public:
	inline std::vector<CSegParams> GetNewSgs() const {
		return oMXY.GetNewSgs();
	}

	inline ~CMarking() {}

	inline CMarking(int _iW) :oMXY(_iW), iW(_iW) {
		x = y = 0;
	}

	inline void Marking(const bool& bBin) {
		if (!x) {
			oMXY.Swap();
		}
		oMXY.Marking(bBin, x, y);
		++x;
		if (x == iW) {
			x = 0;
			++y;
		}
	}
};

#endif //_CMARKING_H_
