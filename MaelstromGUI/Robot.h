#pragma once
#include "PracticalSocket.h"
#include "config.h"
#include <random>
#include <map>

class Robot {

private:
	char rcv_buffer[300];
	UDPSocket continuous_socket;
	UDPSocket request_socket;
	std::string server_ip;
	std::string client_ip;
	unsigned short continuous_port;
	unsigned short request_port;

	// Robot positions
public:
	float pos[3] = {0,0,0}; // x, y, z
	float angles[3] = { 0,0,0 };
	float speed[3] = { 0,0,0 };
	cv::Point3f coordinates = cv::Point3f(0,0,0);
	// Motor positions
	int motors[8] = { 0,0,0,0,0,0,0,0 };
	// Robot state
	int state = -1;

	std::map<std::string, std::string> state_name;
	std::string current_state;

	bool has_started = false;

	
	/*
	States:
		0 wait
		1 home
		10 ready
		20 init
		21 advanced init
		22 check init
		23 finalize advanced init
		30 all pos
		31 all neg
		32 single pos
		33 single neg
		40 control move
		41 manual move
		42 p2p write file
		50 stop
		100 error
	*/
	// Actuators state
	std::string gripper_state_str = "FALSE";
	std::string pump_state_str = "FALSE";
	int gripper_state = 0;
	int pump_state = 0;

	unsigned short request_id = 1;
	int rcv_request_id = 0;
	
	float target[4] = { 0,0,0,0 }; // x, y, z, delta z

	float scanning_depth;

	// Plateform: working zone 
	int max_x = 7; // Shall be divised by two, the 0 is in the center
	int max_y = 4; // Shall be divised by two, the 0 is in the center
	float start_x = - float(max_x) / 2, start_y = - float(max_y) / 2;
	float end_x = float(max_x) / 2, end_y = float(max_y) / 2;

	

public:
	/*------------------------------
		CONSTRUCTOR / DESTRUCTOR
	-------------------------------*/
	Robot() {
		this->server_ip = GUI_IP;
		this->client_ip = ROBOT_IP;
		this->continuous_port = ROBOT_CONTINUOUS_PORT;
		this->request_port = ROBOT_REQUEST_PORT;

		this->continuous_socket.init();
		this->continuous_socket.setLocalAddressAndPort(this->server_ip, this->continuous_port);
		this->continuous_socket.setBroadcast();

		this->request_socket.init();
		this->request_socket.setLocalAddressAndPort(this->server_ip, this->request_port);
		this->request_socket.setBroadcast();

		std::random_device dev;
		std::mt19937 rng(dev());
		std::uniform_int_distribution<std::mt19937::result_type> dist6(1, 10000); // distribution in range [1, 6]
		this->request_id = dist6(rng);

		this->scanning_depth = 0; // Bottom of the pillars. Water should be around -1.5m. 

		this->state_name["0"] = "Wait";
		this->state_name["1"] = "Home";
		this->state_name["10"] = "Ready";
		this->state_name["20"] = "Init";
		this->state_name["21"] = "Advanced init";
		this->state_name["22"] = "Check init";
		this->state_name["23"] = "Finalize advanced init";
		this->state_name["30"] = "All pos";
		this->state_name["31"] = "All neg";
		this->state_name["32"] = "Single pos";
		this->state_name["33"] = "Single neg";
		this->state_name["40"] = "Control move";
		this->state_name["41"] = "Manual move";
		this->state_name["42"] = "P2P write file";
		this->state_name["50"] = "Stop";
		this->state_name["100"] = "Error";

		
	}

	~Robot() {}

	/*------------------------------
		Receive data continuously
	-------------------------------*/
	void rcvData() {
		this->continuous_socket.recvFrom(this->rcv_buffer, 300, (std::string)client_ip, this->continuous_port);
		
		//log("From Robot: " + (std::string)this->rcv_buffer);

		int ret = sscanf(this->rcv_buffer,
			"*%d;%f;%f;%f;%f;%f;%f;%f;%f;%f;%d;%d;%d;%d;%d;%d;%d;%d;%d;%s;%s",
			&this->rcv_request_id,
			&this->pos[0], &this->pos[1], &this->pos[2],
			&this->angles[0], &this->angles[1], &this->angles[2],
			&this->speed[0], &this->speed[1], &this->speed[2],
			&this->motors[0], &this->motors[1], &this->motors[2], &this->motors[3], &this->motors[4], &this->motors[5], &this->motors[6], &this->motors[7],
			&this->state, &this->gripper_state_str, &this->pump_state_str);

		// Update coordinates
		this->coordinates = cv::Point3f(this->pos[0], this->pos[1], this->pos[2]);

		// Update gripper state
		if (this->gripper_state_str == "TRUE")
			this->gripper_state = 1;
		else
			this->gripper_state = 0;

		// Update pump state
		if (this->pump_state_str == "TRUE")
			this->pump_state = 1;
		else
			this->pump_state = 0;

		// Set mrad in degree
		this->angles[0] = this->angles[0]  * 180 / PI;
		this->angles[1] = this->angles[1]  * 180 / PI;
		this->angles[2] = this->angles[2]  * 180 / PI;

		// Update state
		this->current_state = this->state_name[std::to_string(this->state)];
		ZeroMemory(this->rcv_buffer, 300);

		this->has_started = true;
	}

	/*------------------------------
		Send a command to the Robot. 
		Should be in mm.
	-------------------------------*/
	void sendCommand(float x, float y, float z) {
		char answer[100]; // Robot answer (should repeat the received position

		std::string str2send = "*" + std::to_string(this->request_id) + ";" + std::to_string(x) + ";" + std::to_string(y) + ";" + std::to_string(z) + "#";
		this->request_id++;

		request_socket.sendTo(str2send.c_str(), 100, this->client_ip, request_port);
		log("To Robot: " + str2send);

		request_socket.recvFrom(answer, 100, this->client_ip, request_port);
		log("To Robot (answer): " + str2send);
	}

	void setTarget(cv::Point3d target) {
		this->target[0] = (float) this->pos[0] + target.x;
		this->target[1] = (float) this->pos[1] + target.y;
		this->target[2] = (float) this->pos[2] + target.z;

		// Should consider the depth map
		this->target[3] = 1; // delta_z
	}

	void goToTarget() {
		float x = this->target[0];
		float y = this->target[1];
		float z = this->target[2];
		float delta_z = this->target[3];

		z = 0;
		delta_z = 0.05;
		goTo(x, y, z, delta_z);
	}

	/*------------------------------
		Send a command to the Robot.
		Should be in mm.
		Should be in a thread!
		a: current position
		d: target position
		b__c
		|  | delta z
		a  d
	-------------------------------*/
	void goTo(float x, float y, float z, float delta_z) {
		
		// First position (current position)
		float a_x = this->pos[0], a_y = this->pos[1], a_z = this->pos[2];
		// Second position (first command)
		float b_x = a_x, b_y = a_y, b_z = a_z + delta_z;
		// Thrid position (second command)
		float c_x = x, c_y = y, c_z = a_z + delta_z;
		// Forth position (third command)
		float d_x = x, d_y = y, d_z = z;

		//std::string str2send = "*" + std::to_string(this->request_id) + ";" + std::to_string(d_x) + ";" + std::to_string(y) + ";" + std::to_string(z) + "#";
		
		bool first_command_sent = false, second_command_sent = false, third_command_sent = false;

		std::string first_command  = "*" + std::to_string(this->request_id) + ";" + std::to_string(b_x) + ";" + std::to_string(b_y) + ";" + std::to_string(b_z) + "#";
		std::string second_command = "*" + std::to_string(this->request_id) + ";" + std::to_string(c_x) + ";" + std::to_string(c_y) + ";" + std::to_string(c_z) + "#";
		std::string third_command  = "*" + std::to_string(this->request_id) + ";" + std::to_string(d_x) + ";" + std::to_string(d_y) + ";" + std::to_string(d_z) + "#";

		log("Sending first command...");
		while (this->state != 10) {
			Sleep(100);
			//std::cout << this->state << std::endl;
		}
		first_command = "*" + std::to_string(this->request_id) + ";" + std::to_string(b_x) + ";" + std::to_string(b_y) + ";" + std::to_string(b_z) + "#";
		request_socket.sendTo(first_command.c_str(), 100, this->client_ip, request_port);
		log("To Robot (first command): " + first_command);
		this->request_id++;
		Sleep(250);

		log("Sending second command...");
		while (this->state != 10) {
			Sleep(100);
			//std::cout << this->state << std::endl;
		}
		second_command = "*" + std::to_string(this->request_id) + ";" + std::to_string(c_x) + ";" + std::to_string(c_y) + ";" + std::to_string(c_z) + "#";
		request_socket.sendTo(second_command.c_str(), 100, this->client_ip, request_port);
		log("To Robot (second command): " + second_command);
		this->request_id++;
		Sleep(250);

		log("Sending second command...");
		while (this->state != 10) {
			Sleep(100);
			//std::cout << this->state << std::endl;
		}
		third_command = "*" + std::to_string(this->request_id) + ";" + std::to_string(d_x) + ";" + std::to_string(d_y) + ";" + std::to_string(d_z) + "#";
		request_socket.sendTo(third_command.c_str(), 100, this->client_ip, request_port);
		log("To Robot (third command): " + third_command);
		this->request_id++;
		Sleep(250);
	}

	void sendCommand(cv::Point3d pos) {
		log("Sending command...");
		while (this->state != 10) {
			Sleep(100);
		}
		std::string command = "*" + std::to_string(this->request_id) + ";" + std::to_string(pos.x) + ";" + std::to_string(pos.y) + ";" + std::to_string(pos.z) + "#";
		request_socket.sendTo(command.c_str(), 100, this->client_ip, request_port);
		this->request_id++;
	}

	void scan() {
		// +/-3.5x +/-2y (7m - 4m)

		/*
			  Plateform
			 ___________
			|           |
			|           |		x
			|           |		|
			|           |		|___ y
			|     0     | +- 3.5m
			|           |
			|           |
			|			|
			 -----------
				+- 2.0
		*/
		float z = this->scanning_depth;

		float step = 0.5;
		int n_step = this->max_x / step;

		float x = 0, y = 0;

		std::vector<cv::Point3d> scan_path;

		for (int i = 0; i < n_step+1; i++) {
			if (i == 0) {	
				x = this->start_x;
				y = this->start_y;
				scan_path.push_back(cv::Point3d(x, y, z));
				y = this->end_y;
				scan_path.push_back(cv::Point3d(x, y, z));
			}
			else {
				if (i % 2 != 0) {
					x += step;
					scan_path.push_back(cv::Point3d(x, y, z));
					y = this->start_y;
					scan_path.push_back(cv::Point3d(x, y, z));
				}
				else {
					x += step;
					scan_path.push_back(cv::Point3d(x, y, z));
					y = this->end_y;
					scan_path.push_back(cv::Point3d(x, y, z));
				}
			}
			log(std::to_string(x) + " - " + std::to_string(y));
		}

		for (size_t i = 0; i < scan_path.size(); i++) {
			log("Scanning: " + std::to_string(i) + "/" + std::to_string(scan_path.size()));
			sendCommand(scan_path[i]);
			log(std::to_string(scan_path[i].x) + " - " + std::to_string(scan_path[i].y));

			Sleep(250);
		}

	}

	void goToGPSPos(double lat_orig_carte, double long_orig_carte, double lat_P, double long_P, int nb_cases_cote_carte, int longueur_cote_carte, int which_res) {

		int global_i = 0, global_j = 0;
		calcul_coordonnees_i_j_du_point_P_dans_carte_globale(lat_orig_carte, long_orig_carte,
			lat_P, long_P,
			nb_cases_cote_carte, longueur_cote_carte,
			&global_i, &global_j);



		//goTo()
	}

	/*------------------------------
		GETTER
	-------------------------------*/
	float* getPos() { 
		//loat* pos_in_m = new float[3]
		return this->pos; 
	}

	/*------------------------------
		SETTER
	-------------------------------*/
	void setPos(float x, float y, float z) {
		this->pos[0] = x;
		this->pos[1] = y;
		this->pos[2] = z;
	}

	/*------------------------------
	GPS RELATED
	-------------------------------*/

	double distance_entre_deux_points_geographiques(double lat1, double long1, double lat2, double long2)
	{
		//calcule la distance en mètres entre deux points géographiques exprimés en DD.dddddd.
		double delta = DEG2RAD * (long2 - long1);
		double cdlong = cos(delta);
		lat1 = DEG2RAD * lat1;
		lat2 = DEG2RAD * lat2;
		double slat1 = sin(lat1);
		double clat1 = cos(lat1);
		double slat2 = sin(lat2);
		double clat2 = cos(lat2);
		delta = acos(slat1 * slat2 + clat1 * clat2 * cdlong);
		return delta * RAYON_TERRE;
	}

	double cap_pour_aller_du_point_1_au_point_2(double lat1, double long1, double lat2, double long2)
	{
		// calcule l'angle d'orientation en radians (entre 0 pour le Nord et 2PI) du vecteur allant de la position 1 à la position 2,
		// exprimés en DD.dddddd (signés)
		double dlon = DEG2RAD * (long2 - long1);
		lat1 = DEG2RAD * lat1;
		lat2 = DEG2RAD * lat2;
		double a1 = sin(dlon) * cos(lat2);
		double a2 = sin(lat1) * cos(lat2) * cos(dlon);
		a2 = cos(lat1) * sin(lat2) - a2;
		a2 = atan2(a1, a2);
		if (a2 < 0.0)
		{
			a2 += 2 * PI;
		}
		return a2;  //radians
	}

	void calcul_coordonnees_geo_point(double lat_GPS_babord, double long_GPS_babord, double distance, double azimut, double* lat_P, double* long_P)
	{
		//calcule les coordonnées géographiques d'un point défini par sa distance en mètres et son azimut en radians par rapport à un point geographique (dans notre cas, le GPS bâbord)
		lat_GPS_babord = DEG2RAD * lat_GPS_babord;
		long_GPS_babord = DEG2RAD * long_GPS_babord;
		*lat_P = asin(sin(lat_GPS_babord) * cos(distance / RAYON_TERRE) + cos(lat_GPS_babord) * sin(distance / RAYON_TERRE) * cos(azimut));
		*long_P = long_GPS_babord + atan2(sin(azimut) * sin(distance / RAYON_TERRE) * cos(lat_GPS_babord), cos(distance / RAYON_TERRE) - sin(lat_GPS_babord) * sin(*lat_P));
		*lat_P *= RAD2DEG;
		*long_P *= RAD2DEG;
		return;
	}

	void calcul_coordonnees_i_j_du_point_P_dans_carte_globale(double lat_orig_carte, double long_orig_carte, double lat_P, double long_P, int nb_cases_cote_carte, int longueur_cote_carte, int* i, int* j)
	{
		//origine de la carte en haut à gauche (nord ouest). Axe i orienté vers la droite (est) et Axe j orienté vers le bas (sud)
		//La fonction renvoie les coordonnées i,j dans la matrice (nb_cases_cote_carte x nb_cases_cote_carte), ou bien renvoie -1 si le point est hors de la carte
		double distance_par_rapport_origine_carte = distance_entre_deux_points_geographiques(lat_orig_carte, long_orig_carte, lat_P, long_P);
		double azimut_point_P_par_rapport_origine_carte = cap_pour_aller_du_point_1_au_point_2(lat_orig_carte, long_orig_carte, lat_P, long_P);
		//printf("distance,azimut du point P par rapport a origine carte globale = %lf m , %lf deg", distance_par_rapport_origine_carte, RAD2DEG*azimut_point_P_par_rapport_origine_carte);
		double x_carte_globale = distance_par_rapport_origine_carte * sin(azimut_point_P_par_rapport_origine_carte);
		double y_carte_globale = -distance_par_rapport_origine_carte * cos(azimut_point_P_par_rapport_origine_carte);
		*i = (int)(x_carte_globale * nb_cases_cote_carte / longueur_cote_carte);
		*j = (int)(y_carte_globale * nb_cases_cote_carte / longueur_cote_carte);
		if ((*i < 0) || (*j < 0) || (*i >= nb_cases_cote_carte) || (*j >= nb_cases_cote_carte)) //si le point est hors de la carte
		{
			*i = -1;
			*j = -1;
		}
		return;
	}

	void coordonnees_geo_d_un_point_P_x_y_du_repere_robot(double lat_GPS_babord, double long_GPS_babord, double x, double y, double* lat_P, double* long_P, double cap_GPS_Babord_vers_Tribord)
	{
		//Le GPS bâbord est le Master et est le centre du repère GPS
		double x_dans_repere_gps_babord = x - X_GPS_BABORD_DANS_REPERE_ROBOT;                //Les axes de ce reprèe sont orientés comme ceux du robot. Le centre est sur le poteau du GPS arrière bâbord.
		double y_dans_repere_gps_babord = y - Y_GPS_BABORD_DANS_REPERE_ROBOT;
		double azimut_de_P_dans_repere_GPS_babord = -atan2(x_dans_repere_gps_babord, -y_dans_repere_gps_babord); //Angle compté négatif dans le sens horaire à partir de l'axe -y (i.e. l'axe calculé par la fonction GPS RTK qui mesure l'axe Master -> Slave)
		//printf("azimut local = %lf\n", RAD2DEG*azimut_de_P_dans_repere_GPS_babord);
		double azimut_de_P_dans_repere_geographique = cap_GPS_Babord_vers_Tribord + azimut_de_P_dans_repere_GPS_babord;
		//printf("azimut geographique = %lf\n", RAD2DEG*azimut_de_P_dans_repere_geographique);
		double distance_de_P_par_rapport_au_GPS_babord = sqrt(x_dans_repere_gps_babord * x_dans_repere_gps_babord + y_dans_repere_gps_babord * y_dans_repere_gps_babord);
		//printf("distance du point P par rapport au GPS babord = %lf\n", distance_de_P_par_rapport_au_GPS_babord);
		double latP, longP;
		calcul_coordonnees_geo_point(lat_GPS_babord, long_GPS_babord, distance_de_P_par_rapport_au_GPS_babord, azimut_de_P_dans_repere_geographique, &latP, &longP);
		//printf("Coordonnees geographiques de P = %lf %lf\n", latP, longP);
		*lat_P = latP;
		*long_P = longP;
		return;
	}

	void convertit_coordonnees_x_y_locales_en_i_j_grande_carte(double lat_orig_carte, double long_orig_carte, double latitude_GPS_Master_babord, double longitude_GPS_Master_babord, double cap_GPS_babord_vers_tribord, double x_dans_repere_robot, double y_dans_repere_robot, int* i_carte_globale, int* j_carte_globale)
	{
		//cette fonction renvoie i = -1 et j =-1 si l'une des coordonnées est hors de la carte globale, sinon ça renvoie les coordonnées i,j de la carte globale en fonction des x,y du repère robot.
		//origine de la carte en haut à gauche (nord ouest). Axe i orienté vers la droite (est) et Axe j orienté vers le bas (sud)
		double lat_P_temp, long_P_temp;
		coordonnees_geo_d_un_point_P_x_y_du_repere_robot(latitude_GPS_Master_babord, longitude_GPS_Master_babord, x_dans_repere_robot, y_dans_repere_robot, &lat_P_temp, &long_P_temp, cap_GPS_babord_vers_tribord);
		calcul_coordonnees_i_j_du_point_P_dans_carte_globale(lat_orig_carte, long_orig_carte, lat_P_temp, long_P_temp, NB_CASES_COTE_CARTE_GLOBALE, LONGUEUR_COTE_CARTE_GLOBALE, i_carte_globale, j_carte_globale);
	}

	void convertit_coordonnees_x_y_locales_en_i_j_tres_grande_carte(double lat_orig_carte, double long_orig_carte, double latitude_GPS_Master_babord, double longitude_GPS_Master_babord, double cap_GPS_babord_vers_tribord, double x_dans_repere_robot, double y_dans_repere_robot, int* i_carte_globale, int* j_carte_globale)
	{
		//cette fonction renvoie i = -1 et j =-1 si l'une des coordonnées est hors de la carte globale, sinon ça renvoie les coordonnées i,j de la carte globale en fonction des x,y du repère robot.
		//origine de la carte en haut à gauche (nord ouest). Axe i orienté vers la droite (est) et Axe j orienté vers le bas (sud)
		double lat_P_temp, long_P_temp;
		coordonnees_geo_d_un_point_P_x_y_du_repere_robot(latitude_GPS_Master_babord, longitude_GPS_Master_babord, x_dans_repere_robot, y_dans_repere_robot, &lat_P_temp, &long_P_temp, cap_GPS_babord_vers_tribord);
		calcul_coordonnees_i_j_du_point_P_dans_carte_globale(lat_orig_carte, long_orig_carte, lat_P_temp, long_P_temp, NB_CASES_COTE_CARTE_GLOBALE_VERY_HIGH, LONGUEUR_COTE_CARTE_GLOBALE, i_carte_globale, j_carte_globale);
	}

	/*------------------------------
		Timestamp
	-------------------------------*/
	std::string getTimeStamp() {
		using std::chrono::system_clock;
		auto now = std::chrono::system_clock::now();
		char buffer[80];
		auto transformed = now.time_since_epoch().count() / 1000000;
		auto millis = transformed % 1000;
		std::time_t tt;
		tt = system_clock::to_time_t(now);
		auto timeinfo = localtime(&tt);
		strftime(buffer, 80, "%F", timeinfo);
		std::string timestamp = std::string(buffer) + '_' + time_in_HH_MM_SS_MMM();
		return timestamp;
	}
	void log(std::string log_str) {
		log_str.erase(std::remove(log_str.begin(), log_str.end(), '\n'), log_str.end());
		std::cout << "[" << getTimeStamp() << "]: " << log_str << std::endl;
	}
	std::string time_in_HH_MM_SS_MMM() {
		using namespace std::chrono;
		// get current time
		auto now = system_clock::now();
		// get number of milliseconds for the current second
		// (remainder after division into seconds)
		auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
		// convert to std::time_t in order to convert to std::tm (broken time)
		auto timer = system_clock::to_time_t(now);
		// convert to broken time
		std::tm bt = *std::localtime(&timer);
		std::ostringstream oss;
		oss << std::put_time(&bt, "%H:%M:%S"); // HH:MM:SS
		oss << '.' << std::setfill('0') << std::setw(3) << ms.count();
		return oss.str();
	}
};
