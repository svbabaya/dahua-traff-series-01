#include "tcpServerSource.h"
#include "globals_cam.h"

#include <string>
#include <vector>

//====================== ge prep
void TCPServerClass::ge_prep(std::string &_ge) {
  pthread_mutex_lock(&camApp::global_mutex);
  std::vector<TraffStat> tmpSVec = traffixApp::global_sensors;
  pthread_mutex_unlock(&camApp::global_mutex);
  std::string stmp;
	_ge.reserve(256);
	_ge = "ge ";
  for (size_t ii = 0; ii < tmpSVec.size(); ++ii) {
    _ge += i2str(tmpSVec[ii].id, stmp);
    _ge += (tmpSVec[ii].carDetected) ? "=1," : "=0,";
		_ge += i2str(tmpSVec[ii].totalCounter, stmp);   _ge += ',';
    _ge += i2str(tmpSVec[ii].speed, stmp);          _ge += ',';
    _ge += i2str(tmpSVec[ii].curLength*10, stmp);   _ge += ',';
    _ge += (tmpSVec[ii].carDetected) ? i2str(tmpSVec[ii].gap*10, stmp) : i2str(tmpSVec[ii].occup, stmp);
    _ge += ',';
    _ge += i2str(tmpSVec[ii].k, stmp);
    _ge += ' ';
    }
    *_ge.rbegin() = '\r';
    _ge += '\n';
}

//====================== ge
void TCPServerClass::ge_response(ClientForServ *pCl, const char *msg) {
	std::string _ge;
	ge_prep(_ge);
  data_send(pCl->id_sock, _ge.c_str(), _ge.length(), SND_PACK);
}

//====================== ge2
void TCPServerClass::ge2_response(ClientForServ *pCl, const char *msg) {
  pthread_mutex_lock(&camApp::global_mutex);
	std::vector<TraffStat> tmpSVec = traffixApp::global_sensors;
  pthread_mutex_unlock(&camApp::global_mutex);
	std::string _ge2("ge2 "), stmp;
	_ge2.reserve(256);
	for (size_t ii = 0; ii < tmpSVec.size(); ++ii) {
		_ge2 += i2str(tmpSVec[ii].id, stmp);
		_ge2 += (tmpSVec[ii].carDetected) ? "=1," : "=0,";
		_ge2 += i2str(tmpSVec[ii].curCounter, stmp);   _ge2 += ',';
		_ge2 += i2str(tmpSVec[ii].curSpeed, stmp);     _ge2 += ',';
		_ge2 += i2str(tmpSVec[ii].curLength*10, stmp); _ge2 += ',';
		_ge2 += tmpSVec[ii].curClass;				   _ge2 += ' ';
	}
	*_ge2.rbegin() = '\r';
	_ge2 += '\n';
	data_send(pCl->id_sock, _ge2.c_str(), _ge2.length(), SND_PACK);
}

//====================== gs
void TCPServerClass::gs_response(ClientForServ *pCl, const char *msg) {
  pthread_mutex_lock(&camApp::global_mutex);
  std::vector<TraffStat> tmpSVec = traffixApp::global_sensors;
  pthread_mutex_unlock(&camApp::global_mutex);
  std::string _gs("gs "), stmp;
  _gs.reserve(64);
  for (size_t ii = 0; ii < tmpSVec.size(); ++ii) {
    _gs += i2str(tmpSVec[ii].id, stmp);
    _gs += (tmpSVec[ii].carDetected) ? " 1 " : " 0 ";
  }
  *_gs.rbegin() = '\r';
  _gs += '\n';
  data_send(pCl->id_sock, _gs.c_str(), _gs.length(), SND_PACK);
}

//====================== gk
void TCPServerClass::gk_response(ClientForServ *pCl, const char *msg) {
  pthread_mutex_lock(&camApp::global_mutex);
  std::vector<TraffStat> tmpSVec = traffixApp::global_sensors;
  pthread_mutex_unlock(&camApp::global_mutex);
  std::string _gk("gk "), stmp;
  _gk.reserve(256);
  for (size_t ii = 0; ii < tmpSVec.size(); ++ii)
    {
        _gk += i2str(tmpSVec[ii].id, stmp);         _gk += ' ';
        _gk += i2str(tmpSVec[ii].k, stmp);          _gk += ' ';
        _gk += i2str(tmpSVec[ii].counter, stmp);    _gk += ' ';
        _gk += i2str(tmpSVec[ii].speed, stmp);      _gk += ',';
    }
  *_gk.rbegin() = '\r';
   _gk += '\n';
    data_send(pCl->id_sock, _gk.c_str(), _gk.length(), SND_PACK);
}

//====================== gl
void TCPServerClass::gl_response(ClientForServ *pCl, const char *msg) {
  pthread_mutex_lock(&camApp::global_mutex);
	std::vector<TraffStat> tmpSVec = traffixApp::global_sensors;
  pthread_mutex_unlock(&camApp::global_mutex);
	std::string _gl("gl "), stmp;
	_gl.reserve(128);
	for (size_t ii = 0; ii < tmpSVec.size(); ++ii)
	{
		_gl += i2str(tmpSVec[ii].id, stmp); _gl += " \"";
		_gl += tmpSVec[ii].sensName;        _gl += "\" ";
	}
	*_gl.rbegin() = '\r';
	_gl += '\n';
	data_send(pCl->id_sock, _gl.c_str(), _gl.length(), SND_PACK);
}

//====================== gtrc
void TCPServerClass::gTrC_response(ClientForServ *pCl, const char *msg) {
	gFileCommand(pCl, msg, camApp::global_files.getName_TraffConfig(), "gtrc s=");
}

//====================== gtrp
void TCPServerClass::gTrP_response(ClientForServ *pCl, const char *msg) {
	gFileCommand(pCl, msg, camApp::global_files.getName_TraffParam(), "gtrp s=");
}

//====================== strc
void TCPServerClass::sTrC_response(ClientForServ *pCl, const char *msg) {
	sFileCommand(pCl, msg, camApp::global_files.getName_TraffConfig(), true);
}

//====================== strp
void TCPServerClass::sTrP_response(ClientForServ *pCl, const char *msg) {
	sFileCommand(pCl, msg, camApp::global_files.getName_TraffParam(), true);
}

//====================== rq
void TCPServerClass::rq_response(ClientForServ *pCl, const char *msg) {
  int cur_pos = 3, //пропускаем "rq " часть запроса
  str_sz = strlen(msg)-2, //исключаем "\r\n" часть запроса
  last_pos, id;
  if (str_sz <= cur_pos) {
    const std::string _rq("rq c=0\r\n");
    data_send(pCl->id_sock, _rq.c_str(), _rq.length(), SND_PACK);
    return;
  }
  pthread_mutex_lock(&camApp::global_mutex);
  std::vector<TraffStat> tmpSVec = traffixApp::global_sensors;
  pthread_mutex_unlock(&camApp::global_mutex);
  std::string buf(msg), stmp;
  std::vector<int> ids_tmp;
  ids_tmp.reserve(MAX_CLIENTS);
  size_t fnd_;
  do {
    fnd_ = buf.find(',', cur_pos);
    last_pos = (fnd_ == std::string::npos) ? str_sz : fnd_;
    if (last_pos-cur_pos <= 0) {
      cur_pos = last_pos+1;
      continue;
    }
    stmp = buf.substr(cur_pos, last_pos-cur_pos);
    cur_pos = last_pos+1;
    char *pChk(NULL);
    id = strtol(stmp.c_str(), &pChk, 10);
    if (id == 0 && pChk == stmp.c_str()) {
      continue;
    }
    for (size_t ii = 0; ii < tmpSVec.size(); ++ii) {
      //добавление id сенсора на очистку
      //с проверкой есть ли сенсор с таким id,
      //а также с проверкой не был ли id добавлен ранее
      if (tmpSVec[ii].id == id) {
        if (find(ids_tmp.begin(), ids_tmp.end(), id) == ids_tmp.end()) {
          ids_tmp.push_back(id);
        }
        break;
      }
    }
  } while (fnd_ != std::string::npos);
  if (!ids_tmp.empty()) {
    pthread_mutex_lock(&camApp::global_mutex);
    if (traffixApp::global_clrSensorsFlag) {
			traffixApp::global_clrSensIDs.insert(traffixApp::global_clrSensIDs.begin(), ids_tmp.begin(), ids_tmp.end());
    } else {
			traffixApp::global_clrSensIDs = ids_tmp;
    }
		traffixApp::global_clrSensorsFlag = true;
    pthread_mutex_unlock(&camApp::global_mutex);
    }
  std::string _rq("rq c=");
  _rq += i2str(ids_tmp.size(), stmp);
  _rq += "\r\n";
  data_send(pCl->id_sock, _rq.c_str(), _rq.length(), SND_PACK);
}

//====================== gd
void TCPServerClass::gd_response(ClientForServ *pCl, const char *msg) {
  pthread_mutex_lock(&camApp::global_mutex);
  std::vector<TraffStat> tmpSVec = traffixApp::global_sensors;
  pthread_mutex_unlock(&camApp::global_mutex);
  std::string _gd("gd s="), buf, stmp;
  buf.reserve(1024);
  for (size_t ii = 0; ii < tmpSVec.size(); ++ii) {
    buf += i2str(tmpSVec[ii].id, stmp);         buf += ',';
    buf += tmpSVec[ii].time;                    buf += ',';
    buf += i2str(tmpSVec[ii].period, stmp);     buf += ',';
    buf += i2str(tmpSVec[ii].counter, stmp);    buf += ',';
    buf += i2str(tmpSVec[ii].speed, stmp);      buf += ',';
    buf += i2str(tmpSVec[ii].k, stmp);          buf += ',';
    buf += i2str(tmpSVec[ii].occup, stmp);      buf += ',';
    buf += i2str(tmpSVec[ii].Cnt0, stmp);       buf += ',';
    buf += i2str(tmpSVec[ii].Cnt1, stmp);       buf += ',';
    buf += i2str(tmpSVec[ii].Cnt2, stmp);       buf += ',';
    buf += i2str(tmpSVec[ii].Cnt3, stmp);       buf += ',';
    buf += i2str(tmpSVec[ii].Cnt4, stmp);       buf += ',';
    buf += i2str(tmpSVec[ii].Spd0, stmp);       buf += ',';
    buf += i2str(tmpSVec[ii].Spd1, stmp);       buf += ',';
    buf += i2str(tmpSVec[ii].Spd2, stmp);       buf += ',';
    buf += i2str(tmpSVec[ii].Spd3, stmp);       buf += ',';
    buf += i2str(tmpSVec[ii].Spd4, stmp);       buf += ',';
    buf += i2str(tmpSVec[ii].gap, stmp);        buf += ',';
    buf += i2str(tmpSVec[ii].headway, stmp);    buf += ',';
    buf += i2str(tmpSVec[ii].negcnt, stmp);     buf += ',';
    buf += i2str(tmpSVec[ii].videoOk, stmp);    buf += ',';
    buf += tmpSVec[ii].reserv;                  buf += '\n';
  }
  _gd += i2str(buf.length(), stmp);
  _gd += "\r\n";
  _gd += buf;
  data_send(pCl->id_sock, _gd.c_str(), _gd.length(), SND_PACK);
}
