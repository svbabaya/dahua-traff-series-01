#ifndef TCPSERVER_SOURCE_H
#define TCPSERVER_SOURCE_H

#include "_common.h"

#include <map>
#include <queue>
#include <vector>
#include <string>

#include <errno.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/socket.h>

//#define  SRVTEST_PRINT // Вывод информации при работе
#define SRV_CLNTMAP_THRESH  10000

// Класс TCP сервера
class TCPServerClass {
public:
    TCPServerClass() : iInit(0), srvSocket(-1), epollFD(-1),
                       thsConn(NULL), thsWork(NULL), pClients(NULL) {
        clntRcvTimeout.tv_sec  = 0; // {0, 1}  {сек, микросек}
        clntRcvTimeout.tv_usec = 1; // некоторая минимальная задержка
    };
    ~TCPServerClass();
private:
    TCPServerClass(const TCPServerClass&);
	TCPServerClass& operator=(const TCPServerClass&);

    //структура-клиент
    struct ClientForServ {
        struct timespec t_resp;
        bool sm_mode;
        int id_sock;
        ClientForServ();
        ~ClientForServ();
        int trylock(); // ==0 при успехе
        void unlock();
    private:
        pthread_mutex_t mutex;
        ClientForServ(const ClientForServ&);
        ClientForServ& operator=(const ClientForServ&);
    };

    //структура-поток
    struct ThreadForServ {
        #define THR_SRV_STOP    -1
        #define THR_SRV_NONE    -2
        pthread_t tid;
        int id;
        TCPServerClass *parent;
        ThreadForServ()
            : id(THR_SRV_NONE) {};
    };

	typedef void (TCPServerClass::*FunctionResp)(ClientForServ*, const char*);
	typedef void (TCPServerClass::*FunctionIncorrResp)(int, const char*) const;
    typedef std::map<int, ClientForServ*> clnt_map;
    typedef std::map<std::string, FunctionResp> resp_map;

// ПЕРЕМЕННЫЕ, НЕ МЕНЯЮЩИЕСЯ ПРИ РАБОТЕ (mutex is NOT required)
    int MAX_CLIENTS,  // максимальное число клиентов
        THR_CONN_NUM, // кол-во потоков на подключение
        THR_WORK_NUM, // кол-во потоков на обработку запросов
        SND_PACK,     // размер пакетов для отправки (байт)
        DISC_TIME_S;  // максимальное время простоя клиента (сек)
    //время SO_RCVTIMEO для приема данных от клента (задано минимальное)
    struct timeval clntRcvTimeout;// = {0, 1}; //{сек, микросек}

    // для приема команд с данными
    int RCV_DATAPACK_TIME_MS, // макс. задержка между пакетами (милисек)
        RCV_DATA_TIME_S,      // макс. ожидание приема команды и данных (сек)
        RCV_DATA_SIZE;        // макс. размер данных при приеме (байт)
    //флаг готовности к работе
	int iInit; // 0-не запущен, 1-работает, -1-ошибка при запуске
	//порт, сокет, дескриптор для epoll
	int portNo, srvSocket, epollFD;
	// map для выборки функции по строке запроса клиента
    resp_map respMap; ///init в srv_globvar_init
	// поток сервера
	pthread_t srv_tid;
    // потоки-обработчики
    ThreadForServ *thsConn;
    ThreadForServ *thsWork;
    ThreadForServ thSmMode;

// ПЕРЕМЕННЫЕ, МЕНЯЮЩИЕСЯ ПРИ РАБОТЕ (mutex is required)
    //структуры клиентов - выделение памяти при инициализации
    //работа с конкретным клиентом защищается своим мьютексом
    ClientForServ *pClients;      ///ClientForServ::trylock() / unlock()
    //map сокет-клиент
    clnt_map clntMap_W;           ///mutexWork
    //число клиентов
    int clntNum_C;                ///mutexConn
    //последний закрытый клиент
    ClientForServ *pCloseClnt_C;  ///mutexConn
    //число клиентов в авторежиме
    int smClntNum_S;              ///mutexSmMode

  // Средства синхронизации
  //   приоритет (lock order): mutexConn - mutexWork - (ClientForServ mutex) - mutexSmMode
    bool eventConn_C;             ///mutexConn
    pthread_cond_t  condConn_C;   ///mutexConn
    pthread_mutex_t mutexConn_C;

    std::queue<int> eventsWork_W; ///mutexWork
    pthread_cond_t  condWork_W;   ///mutexWork
    pthread_mutex_t mutexWork_W;

    bool flagWakeup_S;            ///mutexSmMode
    pthread_cond_t condWakeup_S;  ///mutexSmMode
    pthread_mutex_t mutexSmMode_S;

public:
    // запуск сервера
    // 0-успешный запуск, 1-уже работает, -1-ошибка при запуске
    //  - port_no			- номер порта
    //  - threads_conn		- кол-во потоков на подключение
    //  - threads_work		- кол-во потоков на обработку запросов
    //  - max_clients		- максимальное число клиентов
    //  - client_idle_sec	- максимальное время простоя клиента (сек)
    //  - send_pack_size	- размер пакетов для отправки (байт)
	//параметры приема команд с данными (например, sc s=):
	//  - rcv_datapack_time_ms	- макс. задержка между пакетами (милисек)
	//  - rcv_data_time_sec		- макс. ожидание приема команды и данных (сек)
	//  - rcv_data_size         - макс. размер данных при приеме (байт)
    int start(int port_no, int threads_conn = 3, int threads_work = 2,
              int max_clients = 5, int client_idle_sec = 30, int send_pack_size = 8192,
			  int rcv_datapack_time_ms = 300, int rcv_data_time_sec = 7, int rcv_data_size = 102400);

    // остановка сервера
    //0-сервер остановлен/не работал
    int stop();

    // проверка работает ли сервер
    bool isWork() const;

    // пробуждение потока отсылки данных в авто режиме
    void smModeWakeup(); ///блокирует mutexSmMode

private:
    // основная функция запуска и работы TCP сервера
    void tcp_server_main();
    static void* srv_thread(void *arg) {
        ((TCPServerClass*)arg)->tcp_server_main();
        return 0;
    };

    // обработчик запросов клиентов на подключение
    void clients_connect(int*const thr_id);
    static void* conn_thread(void *arg) {
        ThreadForServ *tmp = (ThreadForServ*)arg;
        tmp->parent->clients_connect(&(tmp->id));
        return 0;
    };

    // обработчик рабочих запросов клиентов
    void clients_work(int*const thr_id);
    static void* work_thread(void *arg) {
        ThreadForServ *tmp = (ThreadForServ*)arg;
        tmp->parent->clients_work(&(tmp->id));
        return 0;
    };

    // работа с клиентами в авторежиме
    void sm_mode_work(int*const thr_id);
    static void* sm_mode_thread(void *arg) {
        ThreadForServ *tmp = (ThreadForServ*)arg;
        tmp->parent->sm_mode_work(&(tmp->id));
        return 0;
    };

    //инициализация глобальных переменных
    // 0-успешно, иначе - нет
    int srv_globvar_init(int port_no, int threads_conn, int threads_work,
                         int max_clients, int client_idle_sec, int send_pack_size,
						 int rcv_datapack_time_ms, int rcv_data_time_sec, int rcv_data_size);
    //закрытие соединений, чистка ресурсов
    void srv_cleanup();
    //создание прослушивающего сокета
    // возвращает дескриптор или -1 при неудаче
    int srv_socket_init(int port_no) const;
    //создание потоков для работы с клиентами
    // 0-успех, -1-не удалось создать ни одного потока
    int threads_starter();

    //обработка новых подключений к серверу
    void clnt_accept();
    //добавление клиентов в очередь для обработки запросов
    void clnt_data(const std::vector<int> &client_socks);
    //проверка клиентов на простой и отключение
    void clnt_disc(int &epollTimeout, timespec &pre_time, timespec &cur_time);
    //закрытие соединения с клиентом (lock для защиты мьютексами) см.(lock order)
    //  если trylockClnt==true, но заблокировать не удастся, то функция не выполнит никаких действий
    //  рекомендуется обрабатывать Clnt.trylock() извне и передавать trylockClnt=false
    void clnt_close(bool lockConn, bool lockWork, bool lockSmMode, bool trylockClnt, ClientForServ &Clnt);
    //инициализация клиента, если pCl свободен, иначе NULL
    ClientForServ* prepNewClient(ClientForServ *pCl, int client_sock);
    //аутентификация клиента
    bool authentication(int client_sock) const;

    //=========================================
    //функции отправки ответа на запрос клиента
	void conn_lim_resp(int socket) const;
	void incorrect_resp(int socket, const char *msg = NULL) const;
    //=========================================
	///                             !!! ДОБАВЛЕНИЕ КОМАНД В srv_globvar_init !!!
    // параметры: clnt, msg
    // клиент должен быть заблокирован перед вызовом
	// общие
	void gFileCommand(ClientForServ*, const char*, std::string, const char*);
	void sFileCommand(ClientForServ*, const char*, std::string, bool);
    void gv_response(ClientForServ*, const char*);
	void is_response(ClientForServ*, const char*);
	void id_response(ClientForServ*, const char*);
	void gCam_response (ClientForServ*, const char*);
	void sCam_response (ClientForServ*, const char*);	///см. clients_work
	void ex_response(ClientForServ*, const char*);
    void gp_response(ClientForServ*, const char*);
	void sm_response(ClientForServ*, const char*); ///mutexSmMode
	void multi_response(bool lockSmMode); ///mutexSmMode. см. sm_mode_work

	//  traff
	void ge_prep(std::string &_ge);
	void ge_response (ClientForServ*, const char*);
	void ge2_response(ClientForServ*, const char*);
	void gs_response (ClientForServ*, const char*);
	void gk_response (ClientForServ*, const char*);
	void gl_response (ClientForServ*, const char*);
	void gTrC_response (ClientForServ*, const char*);
	void gTrP_response(ClientForServ*, const char*);
    void sTrC_response (ClientForServ*, const char*); ///см. clients_work
	void sTrP_response(ClientForServ*, const char*);  ///см. clients_work
    void rq_response (ClientForServ*, const char*);
	void gd_response (ClientForServ*, const char*);

	//класс для работы с командой, включающей размер данных и сами данные
	class dataCommandWorker {
	private:
		FunctionResp _fcn;
		std::string _cmd, full_data, err;
		int cmd_s_cur, cmd_s_all;
		size_t ch_beg, ch_end;
		int _rcv_allSec;
		struct timespec pre_time;
	public:
		// cmd - искомая команда, после которой идет <число байт>\r\n<данные>
		//fcn - функция ответа на запрос
		//rcv_allsec  - макс. ожидание приема команды и данных (сек)
		dataCommandWorker(std::string cmd, FunctionResp fcn, int rcv_allsec = 5);

		// инициализация перед использованием остальных функций
		void init();

		// сбрасывает работу обработчика, очищает данные
		void resetWorker();

		// поиск позиции cmd в msg
		size_t findCmd(const std::string &msg) const;

		// поиск cmd в msg, выделение числа байт информации,
		//полная команда+данные записывается в full_data
		//обрабатываемая часть информации удаляется из msg
		// -2 - неверно задано число байт после cmd
		// -1 - вышло время ожидания приема
		//  0 - команда не найдена в msg
		//  1 - прием данных в процессе
		//  2 - запрос принят полностью, можно использовать функции ниже
		int procCmd(std::string &msg, int new_num);

		// возвращает full_data - команда+данные
		std::string getCommAndData() const;

		// возвращает сообщение об ошибке, которое формируется, если procCmd вернула -2 или -1
		std::string getLastErr() const;

		// вызывает _fcn (функцию ответа на запрос), если данные были приняты
		void cmdResp(TCPServerClass *_main, ClientForServ *pClnt) const;

		// завершение работы: удаление команды из буфера (true на выходе)
		//если команда уже была удалена ранее, то вернет false
		bool delCmdAndClear(std::string &msg);
	private:
		void clear();
		bool cutData(std::string &msg);
	};

	//класс управления множеством dataCommandWorker
	class dataCommandManager {
	private:
		std::vector<dataCommandWorker> chkVec;
		dataCommandWorker *pCurChk;
		TCPServerClass *_main;
		FunctionIncorrResp _fcnIncorr;
		int _sock;
		struct timeval timeout;
	public:
		//fcnIncorr для отправки сообщения о некорректной команде
		//rcv_delayms - макс. задержка между пакетами (милисек)
		dataCommandManager(FunctionIncorrResp fcnIncorr, TCPServerClass *obj,
						   int rcv_delayms = 100);
		void add(dataCommandWorker wrkObj);
		void init(int socket);
		bool proc(std::string &msg, int new_num, ClientForServ *pClnt); //true, если надо ожидать прием данных
		void delCmds(std::string &msg);
	};
};

// Закрытие соединения с сокетом
inline void closeSocket(int socket) {
    shutdown(socket, SHUT_RDWR);
    close(socket);
}

// Функция отправки данных
inline int data_send(int socket, const void *msg, int length, int packSize) {
    int num, sentNum(0);
    while (sentNum < length) {
        num = length-sentNum > packSize ? packSize : length-sentNum;
        num = send(socket, (char*)msg + sentNum, num, MSG_DONTWAIT);
        if (num > -1) {
            sentNum += num;
        }
        else if (errno == EAGAIN || errno == EWOULDBLOCK) {
            continue;
        }
        else {
            break;
        }
    }
    return sentNum;
}

#endif //TCPSERVER_SOURCE_H
