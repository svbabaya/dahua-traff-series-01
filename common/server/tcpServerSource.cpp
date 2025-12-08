#include "tcpServerSource.h"

#include <syslog.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/epoll.h>
#include <netinet/tcp.h>
#include <netinet/in.h>

template <typename TIn>
void queue_clear(std::queue<TIn> &inp) {
   std::queue<TIn> qemp;
   swap(inp, qemp);
}

TCPServerClass::ClientForServ::ClientForServ()
        : sm_mode(false), id_sock(-1) {
    pthread_mutex_init(&mutex, NULL);
}

TCPServerClass::ClientForServ::~ClientForServ() {
    pthread_mutex_destroy(&mutex);
}

int TCPServerClass::ClientForServ::trylock() {
    return pthread_mutex_trylock(&mutex);
}

void TCPServerClass::ClientForServ::unlock() {
    pthread_mutex_unlock(&mutex);
}

TCPServerClass::~TCPServerClass() {
    stop();
}

int TCPServerClass::start(int port_no, int threads_conn, int threads_work,
						  int max_clients, int client_idle_sec, int send_pack_size,
						  int rcv_datapack_time_ms, int rcv_data_time_sec, int rcv_data_size) {
    if (iInit == 1) {
        return 1;
    }

    //инициализация глобальных переменных
	if (srv_globvar_init(port_no, threads_conn, threads_work, max_clients, client_idle_sec,
                         send_pack_size, rcv_datapack_time_ms, rcv_data_time_sec, rcv_data_size) != 0) {
        srv_cleanup();
        return -1;
    }
    //запуск основного потока сервера
    int res = pthread_create(&srv_tid, NULL, TCPServerClass::srv_thread, this);
    if (res == 0) {
        sched_yield(); //дать потоку заблокироваться
        struct timespec tw = {0, 3000000};
        while (iInit == 0) { //ждем по 3 мс пока поток не инициализируется
            nanosleep(&tw, NULL);
        }
    }

    if (res != 0 || iInit != 1) {
        srv_cleanup();
        return -1;
    }

    return 0;
}

int TCPServerClass::stop() {
	#ifdef SRVTEST_PRINT
        LOGINFO("TcpSrv: stop()\n");
    #endif
    if (iInit != 1) {
        return 0;
    }

    iInit = -1; //сигнал на закрытие
    sched_yield();
    struct timespec tw = {0, 3000000};
    while (iInit != 0) { //ждем по 3 мс пока поток не закроется
        nanosleep(&tw, NULL);
    }

    return 0;
}

bool TCPServerClass::isWork() const {
    if (iInit == 1) {
        return true;
    }
    else {
        return false;
    }
}

void TCPServerClass::smModeWakeup() {
    pthread_mutex_lock(&mutexSmMode_S);
    flagWakeup_S = true;
    pthread_cond_signal(&condWakeup_S);
    pthread_mutex_unlock(&mutexSmMode_S);
}


void TCPServerClass::tcp_server_main() {
    if (iInit == 1) {
        return;
    }

    //создание сокета для связи
    srvSocket = srv_socket_init(portNo);
    if (srvSocket < 0) {
        iInit = -1;
        return;
    }

    pthread_mutex_trylock(&mutexConn_C);   //try?
    pthread_mutex_trylock(&mutexWork_W);   //try?
    pthread_mutex_trylock(&mutexSmMode_S); //try?

    //создаем потоки для обработки запросов на подключение
    //создаем потоки для обработки рабочих запросов
    if (threads_starter() != 0) {
        LOGERR("TcpSrv: ERROR threads create\n");
        iInit = -1;
        return;
    }

    //прослушка на предмет запросов от клиентов
    if (listen(srvSocket, MAX_CLIENTS) != 0) {
        LOGERR("TcpSrv: ERROR listen socket\n");
        iInit = -1;
        return;
    }

    pthread_mutex_unlock(&mutexSmMode_S);
    pthread_mutex_unlock(&mutexWork_W);
    pthread_mutex_unlock(&mutexConn_C);

    //переменные для опроса клиентов, замера времени
    struct epoll_event *epollEvents = new epoll_event[MAX_CLIENTS + 1];
	int epollTimeout(DISC_TIME_S);
    struct timespec cur_time, pre_time;
    //добавляем сокет сервера в набор для проверки запросов на подключение
    {
        struct epoll_event epollEvSrv;
		epollEvSrv.events = EPOLLIN; //| EPOLLONESHOT (!!! но заново добавить сокет после обработки)
		epollEvSrv.data.fd = srvSocket;
		if (epoll_ctl(epollFD, EPOLL_CTL_ADD, srvSocket, &epollEvSrv) == -1)
        {
            LOGERR("TcpSrv: ERROR listen socket\n");
            iInit = -1;
            return;
        }
    }
    //поднимаем флаг готовности к работе данного потока (сервер готов)
    iInit = 1;

    //**************************ЦИКЛ РАБОТЫ СЕРВЕРА**************************
	std::vector<int> sockVec;
	sockVec.reserve(MAX_CLIENTS + 1);
    while ( iInit == 1 ) {
        //проверяем есть ли запросы на подключение или клиентские запросы
        clock_gettime(CLOCK_MONOTONIC, &pre_time);
		int epollNum = epoll_wait(epollFD, epollEvents, MAX_CLIENTS + 1, epollTimeout * 1000);
        if (epollNum > 0) {
			sockVec.clear();
            for (int idx = 0; idx < epollNum; ++idx) {
                //если запрос к сокету сервера
				if (epollEvents[idx].data.fd == srvSocket) {
                    clnt_accept();
                }
                //если запрос от клиента
                else {
					sockVec.push_back(epollEvents[idx].data.fd);
                }
            }
            //клиенты в работу
			clnt_data(sockVec);
        }
        clock_gettime(CLOCK_MONOTONIC, &cur_time);
        #ifdef SRVTEST_PRINT
        LOGINFO("TcpSrv: epoll out\n");
        #endif
        //проверка подключенных клиентов на простой
        clnt_disc(epollTimeout, pre_time, cur_time);
    }

    //завершение работы
	delete[] epollEvents;
    srv_cleanup();
}


int TCPServerClass::srv_globvar_init(int port_no, int threads_conn, int threads_work,
									 int max_clients, int client_idle_sec, int send_pack_size,
									 int rcv_datapack_time_ms, int rcv_data_time_sec, int rcv_data_size) {
    if (iInit == 1 || max_clients < 1 || threads_conn < 1 || threads_work < 1) {
        return -1;
    }

    iInit = 0;
    srvSocket = -1;
    portNo = port_no;

    MAX_CLIENTS = max_clients;
    THR_CONN_NUM = threads_conn;
    THR_WORK_NUM = threads_work;
	SND_PACK = send_pack_size > 32 ? send_pack_size	: 32;
	DISC_TIME_S	= client_idle_sec > 0 ? client_idle_sec : 1;
	RCV_DATAPACK_TIME_MS = rcv_datapack_time_ms > 0 ? rcv_datapack_time_ms : 1;
	RCV_DATA_TIME_S	= rcv_data_time_sec	> 0 ? rcv_data_time_sec	: 1;
	RCV_DATA_SIZE = rcv_data_size > 8 ? rcv_data_size : 8;

    pClients = new ClientForServ[MAX_CLIENTS];
    thsConn = new ThreadForServ[THR_CONN_NUM];
    thsWork = new ThreadForServ[THR_WORK_NUM];

    //данные клиентов
    clntNum_C = 0;
    pCloseClnt_C = NULL;
    //потоки на подключение
    eventConn_C = false;
    for (int idx = 0; idx < THR_CONN_NUM; ++idx)
        thsConn[idx].parent = this;
    //потоки на рабочие запросы
    queue_clear(eventsWork_W);
    for (int idx = 0; idx < THR_WORK_NUM; ++idx) {
        thsWork[idx].parent = this;
    }
    //поток авторежима
    smClntNum_S = 0;
    flagWakeup_S = false;
    thSmMode.id = THR_SRV_NONE;
    thSmMode.parent = this;
    //команды
	//  общие
    respMap["gv\r\n"] = &TCPServerClass::gv_response;
	respMap["is\r\n"] = &TCPServerClass::is_response;
	respMap["id\r\n"] = &TCPServerClass::id_response;
	respMap["gcam\r\n"] = &TCPServerClass::gCam_response;
	//"scam s=" обрабатываются отдельно в clients_work
	respMap["ex\r\n"] = &TCPServerClass::ex_response;
    respMap["gp\r\n"] = &TCPServerClass::gp_response;

	//-команды с параметрами ищутся с символом пробел
	respMap["sm "] = &TCPServerClass::sm_response;

	//  traff
	respMap["ge\r\n"] = &TCPServerClass::ge_response;
	respMap["ge2\r\n"] = &TCPServerClass::ge2_response;
	respMap["gs\r\n"] = &TCPServerClass::gs_response;
    respMap["gk\r\n"] = &TCPServerClass::gk_response;
    respMap["gl\r\n"] = &TCPServerClass::gl_response;
    respMap["gtrc\r\n"] = &TCPServerClass::gTrC_response;
    respMap["gtrp\r\n"] = &TCPServerClass::gTrP_response;
	//"strc s=", "strp s=" обрабатываются отдельно в clients_work

	//-команды с параметрами ищутся с символом пробел
	respMap["rq "] = &TCPServerClass::rq_response;
    respMap["gd "] = &TCPServerClass::gd_response;   //опционально у команды gd
    respMap["gd\r\n"] = &TCPServerClass::gd_response;//может не быть параметров


    //инициализация набора для проверки дескрипторов
    if ( (epollFD = epoll_create(MAX_CLIENTS + 1)) == -1 ) {
        return -1;
    }
    //инициализация мьютексов и условий
    int res = pthread_cond_init(&condConn_C, NULL);
    if (res != 0 && res != EBUSY) {
        return -1;
    }
    res = pthread_mutex_init(&mutexConn_C, NULL);
    if (res != 0 && res != EBUSY) {
        return -1;
    }
    res = pthread_cond_init(&condWork_W, NULL);
    if (res != 0 && res != EBUSY) {
        return -1;
    }
    res = pthread_mutex_init(&mutexWork_W, NULL);
    if (res != 0 && res != EBUSY) {
        return -1;
    }
    res = pthread_mutex_init(&mutexSmMode_S, NULL);
    if (res != 0 && res != EBUSY) {
        return -1;
    }
    res = pthread_cond_init(&condWakeup_S, NULL);
    if (res != 0 && res != EBUSY) {
        return -1;
    }

    return 0;
}

void TCPServerClass::srv_cleanup() {
    pthread_mutex_lock(&mutexConn_C);
    pthread_mutex_lock(&mutexWork_W);
    clntNum_C = MAX_CLIENTS;
    pCloseClnt_C = NULL;
    eventConn_C = false;
    queue_clear(eventsWork_W);
    pthread_mutex_unlock(&mutexWork_W);
    pthread_mutex_unlock(&mutexConn_C);

    struct timespec tw = {0, 3000000};
    //закрытие потока работы с клиентами в авто режиме
     pthread_mutex_lock(&mutexSmMode_S);
    if (thSmMode.id != THR_SRV_NONE) {
        thSmMode.id = THR_SRV_STOP;
    }
    pthread_mutex_unlock(&mutexSmMode_S);
    while (thSmMode.id != THR_SRV_NONE) {
        smModeWakeup();
        nanosleep(&tw, NULL);
    }
    pthread_cond_destroy(&condWakeup_S);
    pthread_mutex_destroy(&mutexSmMode_S);

    //закрытие потоков работы с клиентами (подключение)
    pthread_mutex_lock(&mutexConn_C);
    for (int idx = 0; idx < THR_CONN_NUM; ++idx) {
        if (thsConn[idx].id != THR_SRV_NONE) {
            thsConn[idx].id = THR_SRV_STOP;
        }
    }
    pthread_mutex_unlock(&mutexConn_C);
    for (int idx = 0; idx < THR_CONN_NUM; ++idx) {
        while (thsConn[idx].id != THR_SRV_NONE) {
            pthread_mutex_lock(&mutexConn_C);
            pthread_cond_signal(&condConn_C);
            pthread_mutex_unlock(&mutexConn_C);
            nanosleep(&tw, NULL);
        }
    }
    pthread_cond_destroy(&condConn_C);
    pthread_mutex_destroy(&mutexConn_C);

    //закрытие потоков работы с клиентами (запросы)
    pthread_mutex_lock(&mutexWork_W);
    for (int idx = 0; idx < THR_WORK_NUM ; ++idx) {
        if (thsWork[idx].id != THR_SRV_NONE) {
            thsWork[idx].id = THR_SRV_STOP;
        }
    }
    pthread_mutex_unlock(&mutexWork_W);
    for (int idx = 0; idx < THR_WORK_NUM ; ++idx) {
        while (thsWork[idx].id != THR_SRV_NONE) {
            pthread_mutex_lock(&mutexWork_W);
            pthread_cond_signal(&condWork_W);
            pthread_mutex_unlock(&mutexWork_W);
            nanosleep(&tw, NULL);
        }
    }
    pthread_cond_destroy(&condWork_W);
    pthread_mutex_destroy(&mutexWork_W);

    //закрытие соединений
    for (int idx = 0; idx < MAX_CLIENTS; ++idx) {
        if (pClients[idx].id_sock >= 0) {
            closeSocket(pClients[idx].id_sock);
        }
    }

    respMap.clear();
    clntMap_W.clear();
    clntNum_C = 0;
    pCloseClnt_C = NULL;
    smClntNum_S = 0;
    eventConn_C = false;
    queue_clear(eventsWork_W);

    if (srvSocket >= 0) {
        close(srvSocket);
    }
    srvSocket = -1;
    if (epollFD >= 0) {
        close(epollFD);
    }
    epollFD = -1;
	iInit = 0;

	delete[] pClients;
	pClients = NULL;
	delete[] thsConn;
	thsConn = NULL;
	delete[] thsWork;
	thsWork = NULL;
}

int TCPServerClass::srv_socket_init(int port_no) const {
    if (iInit == 1) {
        return -1;
    }

    //создание сокета для связи
    int _sock = socket(AF_INET, SOCK_STREAM|SOCK_NONBLOCK, 0);
    if (_sock < 0) {
        LOGERR("TcpSrv: ERROR opening socket\n");
        return _sock;
    }

    //инициализируем структуру с параметрами соединения
    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr)); //bzero((char*) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(port_no);

    //возможность переиспользовать данные сетевые настройки, чтобы не ждать пока сокет висит
    //в состоянии TIME_WAIT (что дает ошибку при перезапуске сервера на функции bind ниже)
    int on = 1;
    setsockopt(_sock, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
    setsockopt(_sock, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));

    //связываем созданный сокет с заданными параметрами
    int idx(0), try_num(12);
    for (idx = 0; idx < try_num; ++idx) { //пробуем открыть порт try_num раз (try_num*sleep(5) секунд)
        if (bind(_sock, (struct sockaddr*) &serv_addr, sizeof(serv_addr)) < 0) {
            printf("TcpSrv: Trying to bind the host address...\n");
            sleep(5);
        }
        else {
            break;
        }
    }

    if (idx >= try_num) {
        LOGERR("TcpSrv: ERROR bind the host address\n");
        close(_sock);
        return -1;
    }

    return _sock;
}

int TCPServerClass::threads_starter() {
    if (iInit == 1) {
        return -1;
    }

    thSmMode.id = 0;
    if (pthread_create(&thSmMode.tid, NULL, TCPServerClass::sm_mode_thread, &thSmMode) == 0) {
        sched_yield();
    }
    else {
        thSmMode.id = THR_SRV_NONE;
        return -1;
    }

    int conn_ok(0), work_ok(0);
    int max_iter = (THR_CONN_NUM > THR_WORK_NUM) ? THR_CONN_NUM : THR_WORK_NUM;

    for (int idx = 0; idx < max_iter; ++idx) {
        if (idx < THR_CONN_NUM) {
            thsConn[idx].id = conn_ok;
            if (pthread_create(&thsConn[idx].tid, NULL, TCPServerClass::conn_thread, &thsConn[idx]) == 0) {
                sched_yield();
                ++conn_ok;
            }
            else {
                thsConn[idx].id = THR_SRV_NONE;
            }
        }

        if (idx < THR_WORK_NUM) {
            thsWork[idx].id = work_ok;
            if (pthread_create(&thsWork[idx].tid, NULL, TCPServerClass::work_thread, &thsWork[idx]) == 0) {
                sched_yield();
                ++work_ok;
            }
            else {
                thsWork[idx].id = THR_SRV_NONE;
            }
        }
    }

    if (conn_ok == 0 || work_ok == 0) {
        return -1;
    }

    return 0;
}
