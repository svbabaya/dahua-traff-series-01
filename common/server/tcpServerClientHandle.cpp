#include "tcpServerSource.h"

#include "globals_cam.h"

#include <string.h>
#include <stdlib.h>
#include <sys/epoll.h>

void TCPServerClass::clnt_accept() {
  pthread_mutex_lock(&mutexConn_C);
    #ifdef SRVTEST_PRINT
    LOGINFO("TcpSrv: new accept\n");
    #endif
    eventConn_C = true;
    pthread_cond_signal(&condConn_C);
  pthread_mutex_unlock(&mutexConn_C);
}

void TCPServerClass::clnt_data(const std::vector<int> &client_socks) {
    if (client_socks.empty()) {
        return;
    }
    pthread_mutex_lock(&mutexWork_W);
    for (int ivc = 0; ivc < int(client_socks.size()); ++ivc) {
        eventsWork_W.push(client_socks[ivc]);
        #ifdef SRVTEST_PRINT
        LOGINFO("TcpSrv: new data signal: %i sock\n", client_socks[ivc]);
        #endif
    }
    pthread_cond_signal(&condWork_W);
    pthread_mutex_unlock(&mutexWork_W);
}

void TCPServerClass::clnt_disc(int &epollTimeout, timespec &pre_time, timespec &cur_time) {
    int dtime(cur_time.tv_sec - pre_time.tv_sec);
    if (dtime < epollTimeout) {
        epollTimeout -= dtime;
    }
    else {
        epollTimeout = DISC_TIME_S;
        pthread_mutex_lock(&mutexConn_C);
        if (clntNum_C > 0) {
            // clntNum_C уменьшается при вызове clnt_close
            //          увеличение заблокировано mutexConn
            const int check_all = clntNum_C;
            int checked(0);
            for (int idx = 0; idx < MAX_CLIENTS; ++idx) {
                int lres = pClients[idx].trylock();
                if (lres == EBUSY) {
                    ++checked;
                }
                else if (lres == 0) { //lock section
                    if (pClients[idx].id_sock >= 0) {
                        ++checked;
                        clock_gettime(CLOCK_MONOTONIC, &cur_time);
                        dtime = DISC_TIME_S - int(cur_time.tv_sec - pClients[idx].t_resp.tv_sec);
                        if (dtime <= 0) {
                            #ifdef SRVTEST_PRINT
                            LOGINFO("TcpSrv: idle close: %i sock\n", pClients[idx].id_sock);
                            #endif
                            clnt_close(false, true, true, false, pClients[idx]);
                        }
                        else if (dtime < epollTimeout) {
                            epollTimeout = dtime;
                        }
                    }
                  pClients[idx].unlock();
                }
                if (checked >= check_all) {
                    break;
                }
            } // for по клиентам
        } // clntNum_C > 0
        pthread_mutex_lock(&mutexWork_W);
        #ifdef SRVTEST_PRINT
        LOGINFO("TcpSrv: map size: %i\n", clntMap_W.size());
        #endif
        if ((int(clntMap_W.size()) - clntNum_C) > SRV_CLNTMAP_THRESH) {
            clnt_map tmp;
            clnt_map::const_iterator it = clntMap_W.begin();
            while (it != clntMap_W.end()) {
                if (it->second != NULL) {
                    tmp[it->first] = it->second;
                }
                ++it;
            }
            swap(clntMap_W, tmp);
        } //очистка clntMap_W
        pthread_mutex_unlock(&mutexWork_W);
        pthread_mutex_unlock(&mutexConn_C);
    }
}

// исполнение должно быть всегда защищено мьютексами. см.(lock order) в tcpServerSource.h
void TCPServerClass::clnt_close(bool lockConn, bool lockWork, bool lockSmMode, bool trylockClnt, ClientForServ &Clnt) {
    if (lockConn)   pthread_mutex_lock(&mutexConn_C);
    if (lockWork)   pthread_mutex_lock(&mutexWork_W);
    if (trylockClnt && Clnt.trylock() != 0) {
        if (lockWork)   pthread_mutex_unlock(&mutexWork_W);
        if (lockConn)   pthread_mutex_unlock(&mutexConn_C);
        return;
    }

    if (Clnt.id_sock >= 0) {
        struct epoll_event epollEvEmpty; //for versions before 2.6.9 kernel
		epoll_ctl(epollFD, EPOLL_CTL_DEL, Clnt.id_sock, &epollEvEmpty);
        closeSocket(Clnt.id_sock);
        clntMap_W[Clnt.id_sock] = NULL;
        Clnt.id_sock = -1;
        if (Clnt.sm_mode == true)
        {
            Clnt.sm_mode = false;
          if (lockSmMode)   pthread_mutex_lock(&mutexSmMode_S);
            --smClntNum_S;
          if (lockSmMode)   pthread_mutex_unlock(&mutexSmMode_S);
        }
        --clntNum_C;
        if (pCloseClnt_C == NULL || pCloseClnt_C > &Clnt) {
            pCloseClnt_C = &Clnt;
        }
    }

    #ifdef SRVTEST_PRINT
    LOGINFO("TcpSrv: close client: %i clients, %i sm clients\n", clntNum_C, smClntNum_S);
    #endif
    if (trylockClnt) {
    Clnt.unlock();
    }
    if (lockWork) {
        pthread_mutex_unlock(&mutexWork_W);
    }
    if (lockConn) {
        pthread_mutex_unlock(&mutexConn_C);
    }
}

void TCPServerClass::clients_connect(int*const thr_id) {
    while ( *thr_id >= 0 ) {
      pthread_mutex_lock(&mutexConn_C);
        if (!eventConn_C) {
            pthread_cond_wait(&condConn_C, &mutexConn_C);
        }
        if (*thr_id < 0) {
            break;
        }
        // spurious wakeup
        if (!eventConn_C) {
            pthread_mutex_unlock(&mutexConn_C);
            continue;
        }

        int clnt_sock = accept(srvSocket, NULL, NULL);
        if (clnt_sock < 0) {
            eventConn_C = false;
            pthread_mutex_unlock(&mutexConn_C);
            continue;
        }
        //conn_lim
        if (clntNum_C >= MAX_CLIENTS) {
            #ifdef SRVTEST_PRINT
            LOGINFO("TcpSrv: auth %i: LIM1: %i sock, %i clients\n", *thr_id, clnt_sock, clntNum_C);
            #endif
            conn_lim_resp(clnt_sock);
            closeSocket(clnt_sock);
            clnt_sock = -1;
        }
        #ifdef SRVTEST_PRINT
        else
            LOGINFO("TcpSrv: auth %i: BEG: %i sock, %i clients\n", *thr_id, clnt_sock, clntNum_C);
        #endif

        //в следующий поток. пока очередной accept не вернет -1
        eventConn_C = true;
        pthread_cond_signal(&condConn_C);
        pthread_mutex_unlock(&mutexConn_C);

        //conn_lim
        if (clnt_sock < 0) {
            continue;
        }
        if (authentication(clnt_sock) == false) {
            closeSocket(clnt_sock);
            #ifdef SRVTEST_PRINT
            LOGINFO("TcpSrv: auth %i: FAIL: %i sock\n", *thr_id, clnt_sock);
            #endif
        }
        else {
            ClientForServ *newClnt(NULL);
            pthread_mutex_lock(&mutexConn_C);
            if (clntNum_C < MAX_CLIENTS) {
                if (pCloseClnt_C != NULL) {
                    newClnt = prepNewClient(pCloseClnt_C, clnt_sock);
                    pCloseClnt_C = NULL;
                    #ifdef SRVTEST_PRINT
                    LOGINFO("TcpSrv: auth %i: %i lastP\n", *thr_id, newClnt);
                    #endif
                }
                if (newClnt == NULL) {
                    for (int idx = 0; idx < MAX_CLIENTS; ++idx) {
                        newClnt = prepNewClient(pClients+idx, clnt_sock);
                        if (newClnt != NULL)
                        {
                            #ifdef SRVTEST_PRINT
                            LOGINFO("TcpSrv: auth %i: %i findP\n", *thr_id, newClnt);
                            #endif
                            break;
                        }
                    }
                }
            } //if (clntNum_C < MAX_CLIENTS)
            if (newClnt != NULL) {
                ++clntNum_C;
                #ifdef SRVTEST_PRINT
                LOGINFO("TcpSrv: auth %i: OK: %i sock, %i clients\n", *thr_id, clnt_sock, clntNum_C);
                #endif
            }
            else
            {
                conn_lim_resp(clnt_sock);
                closeSocket(clnt_sock);
                #ifdef SRVTEST_PRINT
                LOGINFO("TcpSrv: auth %i: LIM2: %i sock, %i clients\n", *thr_id, clnt_sock, clntNum_C);
                #endif
            }
            pthread_mutex_unlock(&mutexConn_C);
        } //authentication ok
    } //while( *thr_id >= 0 )

    *thr_id = THR_SRV_NONE;
    pthread_mutex_unlock(&mutexConn_C);
}

TCPServerClass::ClientForServ* TCPServerClass::prepNewClient(ClientForServ *pCl, int client_sock) {
    ClientForServ* out(NULL);
    if (pCl != NULL && pCl->trylock() == 0) { 
        //lock section
        if (pCl->id_sock < 0) {
            out = pCl;
            setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, (void*)&clntRcvTimeout, sizeof(clntRcvTimeout));
            struct epoll_event epollEv;
            epollEv.events = EPOLLIN | EPOLLONESHOT;
            out->id_sock = epollEv.data.fd = client_sock;
            id_response(out, NULL);
            pthread_mutex_lock(&mutexWork_W);
            clntMap_W[client_sock] = out;
            pthread_mutex_unlock(&mutexWork_W);
            clock_gettime(CLOCK_MONOTONIC, &(out->t_resp));
            epoll_ctl(epollFD, EPOLL_CTL_ADD, client_sock, &epollEv);
        }
        pCl->unlock();
    }
    return out;
}

/***** Authentication TCP server */
const std::string tail = "\r\n";
std::string rtrim(const std::string& s) {
    size_t pos = s.find(tail);
    if (pos != std::string::npos) {
        return s.substr(0, pos);
    }
    else {
        return s;
    }
}
bool TCPServerClass::authentication(int client_sock) const {
    // значение таймаута ожидания запроса
    const struct timeval timeout = { 3, 0 }; //{сек, микросек} {DISC_TIME_S, 0}-долго
    // ставим клиентскому сокету таймаут на прием (функция recv будет ждать такое время)
    setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, (void*)&timeout, sizeof(timeout));
    char ch_buf[256];

    // Get login
    // Wait for login timeout
    memset(ch_buf, 0, 256); //bzero(ch_buf, 256);
    if (recv(client_sock, ch_buf, 128, 0) <= 0) {
        return false;
    }
    std::string sLogin(ch_buf); // Login from TCP
    std::string sLoginTrim = rtrim(sLogin);

#ifdef SRVTEST_PRINT
    LOGINFO("> %i sock, login: %s\n", client_sock, ch_buf);
#endif

    // Get password
    // Wait for password timeout
    memset(ch_buf, 0, 256); //bzero(ch_buf, 256);
    if (recv(client_sock, ch_buf, 128, 0) <= 0) {
        return false;
    }
    std::string sPassw(ch_buf); // Password from TCP
    std::string sPasswTrim = rtrim(sPassw);

#ifdef SRVTEST_PRINT
    LOGINFO("> %i sock, passw: %s\n", client_sock, ch_buf);
#endif

    /** Compare login, password from TCP and auth.txt */
    std::string login;
    std::string password;
    camApp::global_files.readAuthFile(true, login, password);
    if ((sLoginTrim.compare(login) == 0) && (sPasswTrim.compare(password) == 0)) {
        return true;
    }
    else {
        return false;
    }
}
/***** Authentication end */

void TCPServerClass::clients_work(int*const thr_id) {
    std::string sBuf, sSub;
	char recv_str[8192];
	sBuf.reserve(32128);
	sSub.reserve(1024);
	memset(recv_str, 0, 8192); //bzero(recv_str, 8192);
	dataCommandManager dataChk(&TCPServerClass::incorrect_resp, this, RCV_DATAPACK_TIME_MS);
	dataChk.add(dataCommandWorker("scam s=", &TCPServerClass::sCam_response, RCV_DATA_TIME_S));
	dataChk.add(dataCommandWorker("strc s=", &TCPServerClass::sTrC_response, RCV_DATA_TIME_S));
	dataChk.add(dataCommandWorker("strp s=", &TCPServerClass::sTrP_response, RCV_DATA_TIME_S));
    struct epoll_event epollEv;
    epollEv.events = EPOLLIN | EPOLLONESHOT;
    while ( *thr_id >= 0 ) {
        //получение сигнала о наличии работы и получение id клиента
        pthread_mutex_lock(&mutexWork_W);
        if (eventsWork_W.empty()) {
            pthread_cond_wait(&condWork_W, &mutexWork_W);
        }
        if ( *thr_id < 0 ) {
            break;
        }
        if (eventsWork_W.empty()) {
            //spurious wakeup
            pthread_mutex_unlock(&mutexWork_W);
            continue;
        }

        const int client_sock = eventsWork_W.front();
        eventsWork_W.pop();

        ClientForServ *pCurClnt(NULL);
        clnt_map::const_iterator it_c = clntMap_W.find(client_sock);
        if (it_c == clntMap_W.end() || it_c->second == NULL) {
            pthread_mutex_unlock(&mutexWork_W);
            continue;
        }
        else {
            pCurClnt = it_c->second;
            pthread_mutex_unlock(&mutexWork_W);
        }
        while (pCurClnt->trylock() != 0) { 
            //блок клиента
        }
        if (client_sock != pCurClnt->id_sock) { 
            //клиент не дошел
            pCurClnt->unlock();
            continue;
        }
        //получение данных
        sBuf.clear();
        int read_cur(0);
        size_t ch_end(0);
		dataChk.init(client_sock);
        while ((read_cur = recv(client_sock, recv_str, 8128, 0)) > 0) {
            recv_str[read_cur] = '\0';
            sBuf.append(recv_str, read_cur);
            #ifdef SRVTEST_PRINT
            LOGINFO("TcpSrv: > work %i: %i sock: %i bcur, %i ball\n", *thr_id, client_sock, read_cur, sBuf.length());
			LOGINFO("TcpSrv: >> %s : %s\n", recv_str, sBuf.c_str());
            #endif
            //проверка наличия команд "cmd s=" и обработка
			if (dataChk.proc(sBuf, read_cur, pCurClnt)) {
				continue; //прием данных
            }
            //остановка чтения по принятию полной команды в конце буфера или по лимиту
            ch_end = sBuf.rfind("\r\n");
            if((sBuf.length() >= RCV_DATA_SIZE) ||
                (ch_end != std::string::npos && ch_end+2 == sBuf.length())) {
                break;
            }
        }
		//удаление команды "cmd s=" из буфера, если по каким-то причинам она не была обработана в цикле
		dataChk.delCmds(sBuf);

        //если вызов recv вернул 0, т.е. клиент отсоединился
        if (read_cur == 0) {
            #ifdef SRVTEST_PRINT
            LOGINFO("TcpSrv: disc close in work: %i sock\n", client_sock);
            #endif
            clnt_close(true, true, true, false, *pCurClnt);
            pCurClnt->unlock();
            continue;
        }

        #ifdef SRVTEST_PRINT
        LOGINFO("TcpSrv: >> work %i: Beg\n", *thr_id);
        #endif
        //обработка данных
        if (sBuf.length() != 0) {
            //возможна ситуация, когда команды были приняты слитно
            //разделение по \r\n
            size_t ch_beg(0);
            if ( (ch_end = sBuf.find("\r\n")) == std::string::npos ) {
                incorrect_resp(client_sock, "<CR><LF> are missing");
            }
            while (ch_end != std::string::npos) {
                sSub = sBuf.substr(ch_beg, ch_end-ch_beg + 2);

                //поиск полных комманд без параметров
                resp_map::const_iterator it_r = respMap.find(sSub);
                if (it_r != respMap.end()) {
                    (this->*it_r->second)(pCurClnt, NULL);
                }
                else {
                    //берем первые 3 символа, проверяем по командам с параметрами
                    it_r = respMap.find(sSub.substr(0,3).c_str());
                    if (it_r != respMap.end()) {
                        (this->*it_r->second)(pCurClnt, sSub.c_str());
                    }
                    else {
                        incorrect_resp(client_sock, sSub.substr(0,8).c_str());
                    }
                }
                //если была выполнена команда ex, то sock будет == -1
                if (pCurClnt->id_sock < 0) {
                    break;
                }
                //определяем границы следующей команды
                ch_beg = ch_end + 2;
                ch_end = sBuf.find("\r\n", ch_beg);
            }
        }

        //освобождаем клиента
        if (pCurClnt->id_sock >= 0) { // если не ex
            clock_gettime(CLOCK_MONOTONIC, &pCurClnt->t_resp);
            epollEv.data.fd = pCurClnt->id_sock;
            epoll_ctl(epollFD, EPOLL_CTL_MOD, pCurClnt->id_sock, &epollEv);
        }
        pCurClnt->unlock();

        #ifdef SRVTEST_PRINT
        LOGINFO("TcpSrv: >> work %i: End\n", *thr_id);
        #endif
    }

    *thr_id = THR_SRV_NONE;
    pthread_mutex_unlock(&mutexWork_W);
}

TCPServerClass::dataCommandManager::dataCommandManager(FunctionIncorrResp fcnIncorr, TCPServerClass *obj,
													   int rcv_delayms/*=100*/)
	: pCurChk(NULL), _main(obj), _fcnIncorr(fcnIncorr), _sock(-1) {
	timeout.tv_sec = int(rcv_delayms / 1000);
	timeout.tv_usec = (rcv_delayms - int(timeout.tv_sec * 1000)) * 1000;
}

void TCPServerClass::dataCommandManager::add(dataCommandWorker wrkObj) {
	chkVec.push_back(wrkObj);
}

void TCPServerClass::dataCommandManager::init(int socket) {
	pCurChk = NULL;
	_sock = socket;
	for (int ivc = 0; ivc < int(chkVec.size()); ++ivc) {
		chkVec[ivc].init();
    }
}

bool TCPServerClass::dataCommandManager::proc(std::string &msg, int new_num, ClientForServ *pClnt) {
	bool firstTime(false);
	int cmdCount(0);
	if (pCurChk == NULL) {
		size_t minPos(std::string::npos);
		for (int ivc = 0; ivc < int(chkVec.size()); ++ivc) {
			size_t pos = chkVec[ivc].findCmd(msg);
			if (pos == std::string::npos) {
				continue;
            }
			firstTime = true;
			++cmdCount;
			if (pos < minPos) {
				pCurChk = &chkVec[ivc];
				minPos = pos;
			}
		}
	}
	if (pCurChk == NULL) {
		return false;
    }
	switch (pCurChk->procCmd(msg, new_num)) {
	case 1:
		if (firstTime == true) {
            //устанавливаем ожидание приема данных
			setsockopt(_sock, SOL_SOCKET, SO_RCVTIMEO, (void*)&timeout, sizeof(timeout));
        }
		return true;
	case -2:
	case -1:
		if (firstTime == false) {
			setsockopt(_sock, SOL_SOCKET, SO_RCVTIMEO, (void*)&(_main->clntRcvTimeout), sizeof(_main->clntRcvTimeout));
        }
		(_main->*_fcnIncorr)(_sock, pCurChk->getLastErr().c_str());
		break;
	case 2:
		if (firstTime == false) {
			setsockopt(_sock, SOL_SOCKET, SO_RCVTIMEO, (void*)&(_main->clntRcvTimeout), sizeof(_main->clntRcvTimeout));
        }
		pCurChk->cmdResp(_main, pClnt);
		pCurChk->resetWorker();
		break;
	default:
		break;
	}
	pCurChk = NULL;
	return (cmdCount > 1);
}

void TCPServerClass::dataCommandManager::delCmds(std::string &msg) {
	if (pCurChk && pCurChk->delCmdAndClear(msg)) {
		setsockopt(_sock, SOL_SOCKET, SO_RCVTIMEO, (void*)&(_main->clntRcvTimeout), sizeof(_main->clntRcvTimeout));
		(_main->*_fcnIncorr)(_sock, pCurChk->getLastErr().c_str());
	}
	pCurChk = NULL;
}

TCPServerClass::dataCommandWorker::dataCommandWorker(std::string cmd, FunctionResp fcn, int rcv_allsec/*=5*/)
	: _fcn(fcn), _cmd(cmd), cmd_s_all(0), _rcv_allSec(rcv_allsec) {

    }

void TCPServerClass::dataCommandWorker::init() {
	resetWorker();
}

void TCPServerClass::dataCommandWorker::resetWorker() {
	clear();
	full_data.clear();
}

void TCPServerClass::dataCommandWorker::clear() {
    ch_beg = ch_end = cmd_s_cur = cmd_s_all = 0;
}

bool TCPServerClass::dataCommandWorker::cutData(std::string &msg) {
    if (cmd_s_cur >= cmd_s_all) {
        size_t str_size = cmd_s_all + (ch_end - ch_beg) + 2;
        full_data = msg.substr(ch_beg, str_size);
        msg.erase(ch_beg, str_size);
        return true;
    }
    else {
        return false;
    }
}

size_t TCPServerClass::dataCommandWorker::findCmd(const std::string &msg) const {
	size_t c1 = msg.find(_cmd);
	if (c1 != std::string::npos && msg.find("\r\n", c1) != std::string::npos) {
		return c1;
    }
	else {
		return std::string::npos;
    }
}

int TCPServerClass::dataCommandWorker::procCmd(std::string &msg, int new_num) {
    //все данные были приняты, сообщаем об этом
    if (!full_data.empty()) {
        return 2;
    }
    //команда не была найдена ранее
    if (cmd_s_all <= 0) {
        if( (ch_beg = msg.find(_cmd))           != std::string::npos &&
            (ch_end = msg.find("\r\n", ch_beg)) != std::string::npos) {
            //выделяем из запроса кол-во байт файла //atoi(.c_str())
            cmd_s_all = strtol(msg.substr(ch_beg+_cmd.length(), ch_end-(ch_beg+_cmd.length())).c_str(), NULL, 10);
            if (cmd_s_all > 0) {
                //сколько принято байт после "_cmd...\r\n"
                cmd_s_cur = msg.length() - (ch_end+2);
                if (cutData(msg)) {
                    //весь объем данных извлечен
                    clear();
                    return 2;
                }
                else {   
                    //ожидание приема данных
                    //засекаем время с момента поступления команды
                    clock_gettime(CLOCK_MONOTONIC, &pre_time);
                    return 1;
                }
            }
            else {   
                //задано неверное число байт файла
                msg.erase(ch_beg, ch_end-ch_beg+2);
                clear();
                err = _cmd+"inval";
                return -2;
            }
        }
        else {
            return 0;
        }
    }

    // cmd_s_all > 0
    // команда была найдена, но не весь объем данных был принят ранее
    cmd_s_cur += new_num;
    if (cutData(msg)) {
        //весь объем данных извлечен
        clear();
        return 2;
    }

    //всего прием ждем заданное число секунд
	struct timespec cur_time;
    clock_gettime(CLOCK_MONOTONIC, &cur_time);
    if ((cur_time.tv_sec - pre_time.tv_sec) >= _rcv_allSec) {
        //таймаут
        msg.erase(ch_beg, cmd_s_cur + (ch_end - ch_beg) + 2);
        clear();
        err = _cmd+"timeout";
        return -1;
    }
    return 1;
}

std::string TCPServerClass::dataCommandWorker::getCommAndData() const {
    return full_data;
}

std::string TCPServerClass::dataCommandWorker::getLastErr() const {
    return err;
}

void TCPServerClass::dataCommandWorker::cmdResp(TCPServerClass *_main, ClientForServ *pClnt) const {
    if (!full_data.empty()) {
        (_main->*_fcn)(pClnt, full_data.c_str());
    }
}

bool TCPServerClass::dataCommandWorker::delCmdAndClear(std::string &msg) {
	full_data.clear();
    if (cmd_s_all > 0) {
        msg.erase(ch_beg, cmd_s_cur + (ch_end - ch_beg) + 2);
        clear();
        err = _cmd+"timeout2";
        return true;
    }
	else {
		clear();
		return false;
	}
}

void TCPServerClass::sm_mode_work(int*const thr_id) {
    while (*thr_id >= 0) {
      pthread_mutex_lock(&mutexSmMode_S);
        while (!flagWakeup_S) {
		    pthread_cond_wait(&condWakeup_S, &mutexSmMode_S);
        }
		flagWakeup_S = false;
        if (*thr_id < 0) {
            break;
        }
        if (smClntNum_S > 0) {
            #ifdef SRVTEST_PRINT
            LOGINFO("TcpSrv: sm mode %i: IN %i smnum\n", *thr_id, smClntNum_S);
            #endif
            multi_response(false);//рассылка сообщений
        }
        pthread_mutex_unlock(&mutexSmMode_S);
    }
    *thr_id = THR_SRV_NONE;
    pthread_mutex_unlock(&mutexSmMode_S);
}
