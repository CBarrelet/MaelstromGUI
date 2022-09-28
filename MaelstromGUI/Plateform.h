#pragma once
#include "PracticalSocket.h"
#include "config.h"

#include <stdio.h>
#include <time.h>
#include <iostream>
#include <conio.h>
#include <Windows.h>
#include <math.h>





class Plateform {
private:
	char rcv_buffer_S[300]; // Starboard (tribord)
	char rcv_buffer_P[300]; // Port (babord)

	char rcv_buffer_GPS_S[300]; // Starboard (tribord)
	char rcv_buffer_GPS_P[300]; // Port (babord)
	char rcv_buffer_GPS[300]; // GPS data

	UDPSocket continuous_S_socket; 
	UDPSocket continuous_P_socket; 
	UDPSocket continuous_GPS_S_socket;
	UDPSocket continuous_GPS_P_socket;

	UDPSocket continuous_GPS_socket;

	std::string server_ip;
	std::string client_ip;

	unsigned short continuous_S_port;
	unsigned short continuous_P_port;
	unsigned short continuous_GPS_S_port;
	unsigned short continuous_GPS_P_port;
	unsigned short continuous_GPS_port;

public:

	double utm_target_north = 0;
	double utm_target_east = 0;
	double target_cap = 0;

	float atm_pressure = 0;

	float imu_S[3] = { 0, 0, 0 };		// roll, pitch, yaw (degree)
	float v_gyro_S[3] = { 0, 0, 0 };	// Gyro x, y, z  (deg.s^-1)
	float gps_S[2] = { 0, 0 };			// Latitude, longitude
	bool gps_rtk_S = false;				// Valid measure

	float imu_P[3] = { 0, 0, 0 };
	float v_gyro_P[3] = { 0, 0, 0 };
	float gps_P[2] = { 0, 0 }; 
	bool gps_rtk_P = false;

	// Values have to be saved
	double lat_orig_carte = 0, long_orig_carte = 0;  //initialisés à 0 pour que l'on puisse le détecter et n'affecter une valeur qu'une seule fois //coordonnées géographiques du coin haut gauche de la carte
	bool gps_is_init = false;

	double latitude_GPS_Master_babord = 0, longitude_GPS_Master_babord = 0, altitude_GPS_Master_babord = 0, latitude_GPS_Slave_tribord = 0, longitude_GPS_Slave_tribord = 0, altitude_GPS_Slave_tribord = 0;

	double cap_GPS_babord_vers_tribord = 0;

	int i_carte_globale = 0, j_carte_globale = 0;
	int RTK_S = 0;
	double x_dans_repere_robot = 0;
	double y_dans_repere_robot = 0;

public:
	/*------------------------------
		CONSTRUCTOR / DESTRUCTOR
	-------------------------------*/

	Plateform() {
		this->server_ip = LOCAL_IP;
		this->client_ip = LOCAL_IP;
		this->continuous_S_port = PLATEFORM_S_CONTINUOUS_PORT;
		this->continuous_P_port = PLATEFORM_P_CONTINUOUS_PORT;

		this->continuous_GPS_S_port = PLATEFORM_GPS_S_CONTINUOUS_PORT;
		this->continuous_GPS_P_port = PLATEFORM_GPS_P_CONTINUOUS_PORT;

		this->continuous_GPS_port = PLATEFORM_GPS_CONTINUOUS_PORT;

		this->continuous_S_socket.init();
		this->continuous_S_socket.setLocalAddressAndPort(this->server_ip, this->continuous_S_port);
		this->continuous_S_socket.setBroadcast();

		this->continuous_P_socket.init();
		this->continuous_P_socket.setLocalAddressAndPort(this->server_ip, this->continuous_P_port);
		this->continuous_P_socket.setBroadcast();

		this->continuous_GPS_S_socket.init();
		this->continuous_GPS_S_socket.setLocalAddressAndPort(this->server_ip, this->continuous_GPS_S_port);
		this->continuous_GPS_S_socket.setBroadcast();

		this->continuous_GPS_P_socket.init();
		this->continuous_GPS_P_socket.setLocalAddressAndPort(this->server_ip, this->continuous_GPS_P_port);
		this->continuous_GPS_P_socket.setBroadcast();

		this->continuous_GPS_socket.init();
		this->continuous_GPS_socket.setLocalAddressAndPort(this->server_ip, this->continuous_GPS_port);
		this->continuous_GPS_socket.setBroadcast();
	}

	~Plateform() {
		/*this->continuous_S_socket.disconnect();
		this->continuous_P_socket.disconnect();*/
	}

	void set_utm_target(double northing, double easting, double cap) {
		this->utm_target_north = northing;
		this->utm_target_east = easting;
		this->target_cap = cap;
	}

	double distanceGPS_P_S() {
		double distance = 0;
		distance = distance_entre_deux_points_geographiques(this->latitude_GPS_Master_babord, this->longitude_GPS_Master_babord, this->latitude_GPS_Slave_tribord, this->longitude_GPS_Slave_tribord);
		return distance;
	}

	/*------------------------------
		Receive data continuously
	-------------------------------*/

	// Receive data from starboard GPS
	void rcvDataGps() {
		ZeroMemory(this->rcv_buffer_GPS, 300);
		this->continuous_GPS_socket.recvFrom(this->rcv_buffer_GPS, 300, (std::string)client_ip, this->continuous_GPS_port);
		//log("From GPS: " + (std::string)this->rcv_buffer_GPS);
		decode_received_data(this->rcv_buffer_GPS);
	}


	void compute_absolute_ij_coo(float robot_x, float robot_y) {
		convertit_coordonnees_x_y_locales_en_i_j_grande_carte(this->lat_orig_carte, this->long_orig_carte, this->latitude_GPS_Master_babord, this->longitude_GPS_Master_babord, this->cap_GPS_babord_vers_tribord, (double)robot_x, (double)robot_y, &this->i_carte_globale, &this->j_carte_globale);
		printf("(x_robot,y_robot) = (i_carte_globale,j_carte_globale)  ==>  (%.2f,%.2f) = (%d,%d)\n", robot_x, robot_y, this->i_carte_globale, this->j_carte_globale);
	}

	// Receive data from starboard GPS
	void rcvDataGpsS() {
		ZeroMemory(this->rcv_buffer_GPS_S, 300);
		this->continuous_GPS_S_socket.recvFrom(this->rcv_buffer_GPS_S, 300, (std::string)client_ip, this->continuous_GPS_S_port);
		//log("From Plateform GPS S: " + (std::string)this->rcv_buffer_GPS_S);
	}

	// Receive data from port GPS
	void rcvDataGpsP() {
		ZeroMemory(this->rcv_buffer_GPS_P, 300);
		this->continuous_GPS_P_socket.recvFrom(this->rcv_buffer_GPS_P, 300, (std::string)client_ip, this->continuous_GPS_P_port);
		//log("From Plateform GPS S: " + (std::string)this->rcv_buffer_GPS_P);

	}

	// Receive data from starboard side (imu and barometer)
	void rcvDataS() {
		ZeroMemory(this->rcv_buffer_S, 300);
		this->continuous_S_socket.recvFrom(this->rcv_buffer_S, 300, (std::string)client_ip, this->continuous_S_port);
		//log("From Plateform S IMU: " + (std::string)this->rcv_buffer_S);

		float roll = 0, pitch = 0, yaw = 0, depth = 0, gyrx = 0, gyry = 0, gyrz = 0;
		int atm_pressure = 0;

		int ret = sscanf(this->rcv_buffer_S, "A,%f,%f,%f,%f,%f,%f,%d", &roll, &pitch, &yaw, &gyrx, &gyry, &gyrz, &atm_pressure);

		this->atm_pressure = atm_pressure;
		this->imu_S[0] = roll;
		this->imu_S[1] = pitch;
		this->imu_S[2] = yaw;

		this->v_gyro_S[0] = gyrx;
		this->v_gyro_S[1] = gyry;
		this->v_gyro_S[2] = gyrz;
	}

	// Receive data from port side (imu and barometer)
	void rcvDataP() {
		ZeroMemory(this->rcv_buffer_P, 300);
		this->continuous_P_socket.recvFrom(this->rcv_buffer_P, 300, (std::string)client_ip, this->continuous_P_port);
		//log("From Plateform P IMU: " + (std::string)this->rcv_buffer_P);

		float roll = 0, pitch = 0, yaw = 0, depth = 0, gyrx = 0, gyry = 0, gyrz = 0;
		int atm_pressure = 0;

		int ret = sscanf(this->rcv_buffer_P, "B,%f,%f,%f,%f,%f,%f,%d", &roll, &pitch, &yaw, &gyrx, &gyry, &gyrz, &atm_pressure);

		//this->atm_pressure = atm_pressure;
		this->imu_P[0] = roll;
		this->imu_P[1] = pitch;
		this->imu_P[2] = yaw;

		this->v_gyro_P[0] = gyrx;
		this->v_gyro_P[1] = gyry;
		this->v_gyro_P[2] = gyrz;
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

	void getGeoFromXYPos(double x, double y, double* lat_P, double* long_P) {

		double lat_GPS_babord = this->latitude_GPS_Master_babord;
		double long_GPS_babord = this->longitude_GPS_Master_babord;
		double cap_GPS_Babord_vers_Tribord = this->cap_GPS_babord_vers_tribord;
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
	}

	// GPS RELATED
	double convertir_coord_gps_Tecnalia_en_centiemes_de_degres(double valeur_a_convertir)
	{
		//Les coordonnées GPS transmises en UDP par le soft Tecnalia sont sous la forme : DD.MMmmmm.
		//On doit donc convertir la partie minutes (MMmmmm) en centièmes de degrés pour obtenir un nombre de la forme DD.ddddd.
		double partie_entiere = partie_entiere = (double)(int)(valeur_a_convertir);
		return partie_entiere + (valeur_a_convertir - partie_entiere) * 10 / 6;
	}

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

	void definir_coordonnees_geo_origine_carte_globale(double lat_GPS_babord, double long_GPS_babord, double* lat_orig_carte, double* long_orig_carte)
	{
		//calcule les coordonnees geo du coin haut gauche de la carte, définie à -LONGUEUR_COTE_CARTE_GLOBALE/2 au nord, -LONGUEUR_COTE_CARTE_GLOBALE/2 vers l'ouest
		// https://www.dcode.fr/calcul-coordonnees-geographiques
		lat_GPS_babord = DEG2RAD * lat_GPS_babord;
		long_GPS_babord = DEG2RAD * long_GPS_babord;
		double distance_origine_carte = LONGUEUR_COTE_CARTE_GLOBALE * 0.707; //distance à laquelle se trouve l'origine de la carte (taille carte * racine de 2 sur 2)
		double azimut_origine_carte = DEG2RAD * 315; //direction Nord-Ouest = 315 deg.
		*lat_orig_carte = asin(sin(lat_GPS_babord) * cos(distance_origine_carte / RAYON_TERRE) + cos(lat_GPS_babord) * sin(distance_origine_carte / RAYON_TERRE) * cos(azimut_origine_carte));
		*long_orig_carte = long_GPS_babord + atan2(sin(azimut_origine_carte) * sin(distance_origine_carte / RAYON_TERRE) * cos(lat_GPS_babord), cos(distance_origine_carte / RAYON_TERRE) - sin(lat_GPS_babord) * sin(*lat_orig_carte));
		*lat_orig_carte *= RAD2DEG;
		*long_orig_carte *= RAD2DEG;
		//printf("Coordonnees geographiques origine carte georeferencee : %lf %lf", *lat_orig_carte, *long_orig_carte);
		return;
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
		return;
	}

	int decode_received_data(char buffer[128])
	{
		
		char* pointeur_chaine_master = NULL;
		char* pointeur_chaine_slave = NULL;
		char* pointeur_chaine_RTK_master = NULL;
		char* pointeur_chaine_RTK_slave = NULL;
		pointeur_chaine_master = strstr(buffer, "Master");
		pointeur_chaine_slave = strstr(buffer, "Slave");
		pointeur_chaine_RTK_master = strstr(buffer, "RTK");
		if (pointeur_chaine_RTK_master != NULL) pointeur_chaine_RTK_slave = strstr(pointeur_chaine_RTK_master, "RTK");
		if (pointeur_chaine_RTK_slave != NULL) //on vérifie que le GPS slave est bien passé aussi en mode RTK, sinon la précision ne sera pas du tout suffisante.
		{
			this->RTK_S = 1;
			//printf("RTK OK");
			if ((pointeur_chaine_master != NULL) && (pointeur_chaine_slave != NULL))
			{
				//lecture de la trame envoyée en UDP par le soft GPS de Tecnalia
				//printf("\nReception GPS :%s\n", pointeur_chaine_master);
				sscanf(pointeur_chaine_master, "Master,%lf,%lf,%lf", &this->latitude_GPS_Master_babord, &this->longitude_GPS_Master_babord, &this->altitude_GPS_Master_babord);
				this->latitude_GPS_Master_babord = convertir_coord_gps_Tecnalia_en_centiemes_de_degres(this->latitude_GPS_Master_babord);
				this->longitude_GPS_Master_babord = convertir_coord_gps_Tecnalia_en_centiemes_de_degres(this->longitude_GPS_Master_babord);
				sscanf(pointeur_chaine_slave, "Slave,%lf,%lf,%lf", &this->latitude_GPS_Slave_tribord, &this->longitude_GPS_Slave_tribord, &this->altitude_GPS_Slave_tribord);
				this->latitude_GPS_Slave_tribord = convertir_coord_gps_Tecnalia_en_centiemes_de_degres(this->latitude_GPS_Slave_tribord);
				this->longitude_GPS_Slave_tribord = convertir_coord_gps_Tecnalia_en_centiemes_de_degres(this->longitude_GPS_Slave_tribord);
				//printf("Master = %.9lf deg , %.9lf deg\n", latitude_GPS_Master_babord, longitude_GPS_Master_babord);
				//printf("Slave = %.9lf deg , %.9lf deg\n", latitude_GPS_Slave_tribord, longitude_GPS_Slave_tribord);
				//printf("distance calculee entre les deux antennes GPS = %lf m (doit etre 7.79m a quelques cm pres).\n", distance_entre_deux_points_geographiques(latitude_GPS_Master_babord, longitude_GPS_Master_babord, latitude_GPS_Slave_tribord, longitude_GPS_Slave_tribord));
				this->cap_GPS_babord_vers_tribord = cap_pour_aller_du_point_1_au_point_2(this->latitude_GPS_Master_babord, this->longitude_GPS_Master_babord, this->latitude_GPS_Slave_tribord, this->longitude_GPS_Slave_tribord);
				//printf("cap du gps babord vers GPS tribord = %lf deg\n", RAD2DEG * cap_GPS_babord_vers_tribord);
				double cap_axe_x_barge = this->cap_GPS_babord_vers_tribord - PI / 2;
				if (cap_axe_x_barge < 0)
				{
					cap_axe_x_barge += 2 * PI;
				}
				//printf("cap de la barge selon axe x = %lf deg\n", RAD2DEG * cap_axe_x_barge);
				if (this->lat_orig_carte == 0) definir_coordonnees_geo_origine_carte_globale(this->latitude_GPS_Master_babord, this->longitude_GPS_Master_babord, &this->lat_orig_carte, &this->long_orig_carte);  //on ne le fait qu'une seule fois
				//printf("doordonnees geographiques origine carte globale = %.9lf , %.9lf\n", this->lat_orig_carte, this->long_orig_carte);
				//printf("distance entre antenne babord et origine carte = %lf m\n", distance_entre_deux_points_geographiques(latitude_GPS_Master_babord, longitude_GPS_Master_babord, this->lat_orig_carte, this->long_orig_carte));
				//printf("cap de antenne babord vers origine carte= %lf deg\n", RAD2DEG * cap_pour_aller_du_point_1_au_point_2(latitude_GPS_Master_babord, longitude_GPS_Master_babord, this->lat_orig_carte, this->long_orig_carte));
				//double lat_P_temp, long_P_temp;
				//this->x_dans_repere_robot = 0;// X_GPS_BABORD_DANS_REPERE_ROBOT + 0;    //DEBUG valeur fixe pour test des fonctions
				//this->y_dans_repere_robot = 0; // Y_GPS_BABORD_DANS_REPERE_ROBOT - 0;    //DEBUG test fixe pour test des fonctions
				
			}
		}
		else
		{
			this->RTK_S = 0;
			//printf("pas de RTK !!\n");
		}

		return 0;
	}

};






