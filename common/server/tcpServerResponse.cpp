#include "tcpServerSource.h"

#include "globals_cam.h"

#include <arpa/inet.h>

//====================== connection limit
void TCPServerClass::conn_lim_resp(int socket) const {
	const std::string _conn_lim("Connection limit reached\r\n");
	data_send(socket, _conn_lim.c_str(), _conn_lim.length(), SND_PACK);
}

//====================== incorrect command
void TCPServerClass::incorrect_resp(int socket, const char *msg /*= NULL*/) const {
	std::string _incorr("Incorrect command");
	if (msg != NULL && strlen(msg) > 0) {
		_incorr += ": ";
		_incorr += msg;
	}
	if (_incorr.find("\r\n") == std::string::npos) {
		_incorr += "\r\n";
  }
	data_send(socket, _incorr.c_str(), _incorr.length(), SND_PACK);
}

//====================== get file data command proc
void TCPServerClass::gFileCommand(ClientForServ *pCl, const char *msg, std::string fName, const char* cmd) {
	std::string data;
	if (false == camApp::global_files.readTextFromFile(true, data, fName)) {
		std::string _gco(cmd);
		_gco += "0\r\n";
		data_send(pCl->id_sock, _gco.c_str(), _gco.length(), SND_PACK);
		return;
	}

	std::string _gco(cmd), stmp;
	_gco += i2str(data.length(), stmp);
	_gco += "\r\n";
	data_send(pCl->id_sock, _gco.c_str(), _gco.length(), SND_PACK);
	data_send(pCl->id_sock, data.c_str(), data.length(), SND_PACK);
}

//====================== set file data command proc
void TCPServerClass::sFileCommand(ClientForServ *pCl, const char *msg, std::string fName, bool glUpdFlag) {
	std::string _sco;
	const char *ch_en = strchr(msg, '\n');
	if (true == camApp::global_files.writeTextToFile(true, std::string(ch_en + 1), fName)) {
		if (glUpdFlag) {
			camApp::global_files.setUpdConfigFlag(true);
    }
		_sco = std::string(msg, ch_en - msg + 1);
	}
	else {
		ch_en = strchr(msg, '=');
		_sco = std::string(msg, ch_en - msg + 1);
		_sco += "0\r\n";
	}

	data_send(pCl->id_sock, _sco.c_str(), _sco.length(), SND_PACK);
}

//====================== gv
void TCPServerClass::gv_response(ClientForServ *pCl, const char *msg) {
	std::string _gv("gv v=\"");
	_gv += camApp::global_gvStr;
	_gv += "\"\r\n";
	data_send(pCl->id_sock, _gv.c_str(), _gv.length(), SND_PACK);
}

//====================== is
void TCPServerClass::is_response(ClientForServ *pCl, const char *msg) {
	std::string _is = camApp::global_initFlag ? "is [INIT OK]\r\n" : "is [INIT NO]\r\n";
	data_send(pCl->id_sock, _is.c_str(), _is.length(), SND_PACK);
}

//====================== id
void TCPServerClass::id_response(ClientForServ *pCl, const char *msg) {
	std::string _id("id [");
	_id += camApp::global_files.getCamId();
	_id += "]\r\n";
	data_send(pCl->id_sock, _id.c_str(), _id.length(), SND_PACK);
}

//====================== gcam
void TCPServerClass::gCam_response(ClientForServ *pCl, const char *msg) {
	gFileCommand(pCl, msg, camApp::global_files.getName_CamConfig(), "gcam s=");
}

//====================== scam
void TCPServerClass::sCam_response(ClientForServ *pCl, const char *msg) {
	sFileCommand(pCl, msg, camApp::global_files.getName_CamConfig(), true);
}

//====================== ex
void TCPServerClass::ex_response(ClientForServ *pCl, const char *msg) {
    clnt_close(true, true, true, false, *pCl);
}

//====================== gp
void TCPServerClass::gp_response(ClientForServ *pCl, const char *msg) {
  pthread_mutex_lock(&camApp::global_mutex);
	if (camApp::global_frame.size() < 1) {
    pthread_mutex_unlock(&camApp::global_mutex);
		const std::string tmp("No frame to send\r\n");
		data_send(pCl->id_sock, tmp.c_str(), tmp.length(), SND_PACK);
	}
	else {
		const int sizeD = camApp::global_frame[0].height() * camApp::global_frame[0].width();
		const size_t byteRow = camApp::global_frame[0].width() * sizeof(camApp::global_frame[0][0][0]);
		unsigned char *pD = new unsigned char[sizeD];
		unsigned char *p = pD;
		for (int yy = 0; yy < camApp::global_frame[0].height(); ++yy) {
			memcpy(p, camApp::global_frame[0][yy], byteRow);
			p += camApp::global_frame[0].width();
		}
    pthread_mutex_unlock(&camApp::global_mutex);
	  std::string _gp("gp s="), stmp;
	  _gp += i2str(sizeD, stmp);
	  _gp += "\r\n";
		data_send(pCl->id_sock, _gp.c_str(), _gp.length(), SND_PACK);
		data_send(pCl->id_sock, pD, sizeD, SND_PACK);
		delete[] pD;
	}
}

//====================== sm
void TCPServerClass::sm_response(ClientForServ *pCl, const char *msg) {
  std::string _sm(msg);
  std::string stmp = _sm.substr(3);//берем все, кроме "sm "
  if (stmp.compare("data=on\r\n") == 0) {
    if (pCl->sm_mode == false) {
      pCl->sm_mode = true;
      pthread_mutex_lock(&mutexSmMode_S);
      ++smClntNum_S;
      pthread_mutex_unlock(&mutexSmMode_S);
    }
    data_send(pCl->id_sock, _sm.c_str(), _sm.length(), SND_PACK);
  }
  else if (stmp.compare("data=off\r\n") == 0) {
    if (pCl->sm_mode == true) {
      pCl->sm_mode = false;
      pthread_mutex_lock(&mutexSmMode_S);
      --smClntNum_S;
      pthread_mutex_unlock(&mutexSmMode_S);
    }
    data_send(pCl->id_sock, _sm.c_str(), _sm.length(), SND_PACK);
  }
  else {
    incorrect_resp(pCl->id_sock, "sm - parameters");
  }
}

//====================== multi_response
void TCPServerClass::multi_response(bool lockSmMode) {
  std::vector<std::string> msgs;
  msgs.push_back(std::string());
  ge_prep(msgs.back());
  if (msgs.empty()) {
    return;
  }
  if (lockSmMode) {
    pthread_mutex_lock(&mutexSmMode_S);
  }
  const int check_all = smClntNum_S;
  int checked(0);
  for (int idx = 0; idx < MAX_CLIENTS; ++idx) {
    if (pClients[idx].trylock() == 0) { //lock section
      if (pClients[idx].sm_mode) {
        ++checked;
        for (size_t ivm = 0; ivm < msgs.size(); ++ivm) {
          if (data_send(pClients[idx].id_sock, msgs[ivm].c_str(), msgs[ivm].length(), SND_PACK) == -1) {
            if (errno == ECONNRESET || errno == ENOTCONN || errno == EPIPE || errno == ENOTSOCK   || errno == EBADF) {
              pClients[idx].sm_mode = false;
              --smClntNum_S;
            }
          }
        }
        clock_gettime(CLOCK_MONOTONIC, &pClients[idx].t_resp);
      }
      pClients[idx].unlock();
    }
    if (checked >= check_all) {
      break;
    }
  }
  if (lockSmMode) {
    pthread_mutex_unlock(&mutexSmMode_S);
  }
}
