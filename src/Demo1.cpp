#include <stdio.h>
#include "/home/pi/Desktop/controlcpp/src/Serial/CommSerialLinux.h"

#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/wait.h>
#include <math.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/signal.h>
#include <sys/time.h>

#include <linux/types.h> 
#include <asm/types.h>
#include <sys/types.h>
#include <fstream>
#include <iostream>
#include <cmath>

#include "armadillo"

#define MAXLINE 255
#define tam 1024
#define BAUDRATE B19200
#define MODEMDEVICE "/dev/ttyS0"
#define _POSIX_SOURCE 1
#define FALSE 0
#define TRUE 1
#undef max
#define max(x,y) ((x) > (y) ? (x) : (y))

using namespace std;
using namespace arma;

//Datos Globales


/*
Variables laterales:Deflexon del aleron
                    Angulo de guiñada (yaw)
                    Angulo de alabeo (roll)
                    Velocidad angular de guiñada (yaw)
                    Velocidad angular de alabeo (roll)
                    Velocidad en el eje y


*/


/*
Variables lONGITUDINAles:Deflexon del elevador
                        Potencia de turbina
                        Angulo de cabeceo (pitch)
                        Velocidad angular de cabeceo(pitch)
                        Velocidad en el eje X
                        Velocidad en el eje Z
                        Altura


*/
int counter1, counter2, counter3;
int titulo;
double Ts=0.027;
const long double pi=3.141516;
float pitch_actual;
float alt_bar_gums;		//Altura Barométrica (m)
int muestra;

typedef struct var_sensor{
float lat_gps; 		//Latitud GPS (Sensor) (grados)
float lon_gps;          //Longitud GPS (Sensor) (grados)
float alt_gps;          //Altitud GPS (Sensor) (m)
float alt_bar;		//Altura Barométrica (m)

float v_gps;            //Velocidad GPS (Sensor) (m/s)
float rumbo_gps;        //Rumbo GPS (Sensor) (grados)
float v_pitot;		//Velocidad de Pitot (Sensor) (m/s)
float rumbo;		//Rumbo Magnético
float rumbo_viento; 	//Dirección del viento
float int_viento;	//Intensidad del viento
float dist_laser;       //Distancia de laser (Sensor) (m)
float ang_giro[3];      //Angulos Pitch, Roll, Yaw Absolutos (Sensor Giroscopo) (grados)
float v_giro[3];        //Velocidad Angular Intrínsica  (Sensor Giroscopo) (rad/seg) [p q r]
float a_giro[3];        //Aceleración Angular Intrínsica (Sensor Giroscopo) (rad/seg^2)
float a_intr[3];        //Aceleración Lineal Intrínsica
float t_aire;		//Sensor de T° avión
float elevador;		//Posicion Servo Elevador
float aleron;		//Posicion Servo Aleron
float timon;		//Posicion Servo Timon
float throttle;		//Posición Servo Valvula
float tiempo;		//Tiempo del Kestrel
} t_var_sensor;

typedef struct lsensor{
	t_var_sensor v_sensor[100];
}t_lsensor;

t_lsensor *lsensor;

//variables calculadas
typedef struct var_calculo{
float p_abs[3];         //Posición Lineal Absoluta (Calculos) (m)
float v_abs[3];         //Velocidad Lineal Absoluta (Calculos) (m/s)
float a_abs[3];         //Aceleración Lineal Absoluta (Calculos) (m/s^2)
float a_giro[3];        //Aceleracion Angular Intrinsica (rad/seg^2)
float v_intr[3];        //Velocidad Lineal Intrínsica  (Calculos) (m/s)
float a_intr[3];        //Aceleración Lineal Intrínsica (Calculos) (m/s^2)
float v_ang_giro[3];    //Velocidad Pitch, Roll, Yaw Absolutas (Calculos) (rad/seg)
float a_ang_giro[3];    //Aceleración Pitch, Roll, Yaw Absolutas (Calculos) (rad/seg^2)
float v_aire[3];        //Velocidad del aire (Calculos) (m/s)
float thrust;           //Empuje (Calculos) (N)
float drag;             //Drag (Calculos)
float lift;             //Lift (Calculos)
float fgrav;            //Fuerza de Gravedad (Calculos)
float alfa;             //Ángulo de ataque (Calculos) (grados)
float beta;             //Angulo deslizamiento lateral (Calculos) (grados)
float gama;             //Heading (Calculos) (grados)
float alt_mapa;         //Altura respecto al mapa cargado (Calculos) (m)
} t_var_calculo;

typedef struct lcalculo{
	t_var_calculo v_calc[100];
}t_lcalculo;

typedef struct cmds{
	int id_cmd;
	int time;
	__u8 roll;
	__u8 pitch;
	__u8 valvula;
}t_cmds;

//////funciones relacionadas con el envio, recibo de datos con kestrel
typedef void (*CallBackFuncPtr)(CCommPacket*, PacketType);
CCommSerialLinux g_CommSerial;
float PitchDes, RollDes, ValvDes;

int abre_puerto_gums(void){
	
	struct termios oldtio,newtio;

	int fd;
        fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY | O_NDELAY);//O_NONBLOCK);
        if(fd <0) {printf("No se logró abrir el puerto\n");perror(MODEMDEVICE); exit(-1); }
        
        fcntl(fd, F_SETFL, FASYNC);
        
        tcgetattr(fd,&oldtio);
 
	cfsetispeed(&newtio, B19200);
	cfsetospeed(&newtio, B19200);

        newtio.c_cflag |= (CLOCAL | CREAD);
	newtio.c_cflag &= ~PARENB;
	newtio.c_cflag &= ~CSTOPB;
	newtio.c_cflag &= ~CSIZE;
	newtio.c_cflag |= CS8;

        newtio.c_iflag = IGNPAR | ICRNL ;
        newtio.c_oflag = ONLCR;
        newtio.c_lflag = ICANON;
        newtio.c_cc[VMIN]= 0;
        newtio.c_cc[VTIME]=0;

        tcflush(fd, TCIFLUSH);
        tcsetattr(fd,TCSANOW,&newtio);
  return(fd);

}

/*
void cargo_cmd(void)
{
	CWriteRawPacket Joy1CommandsPkt(SEND_JOYSTICKS_1, 1032);
	Joy1CommandsPkt.AppendUnsignedChar(RollDes); 	//Comando de Roll
	Joy1CommandsPkt.AppendUnsignedChar(PitchDes); 	//Comando de Pitch
	Joy1CommandsPkt.AppendUnsignedChar(ValvDes);

	g_CommSerial.SendRaw(&Joy1CommandsPkt);
}

*/



void escribir(void)//funcion para observar el como son ingresados los datos a la rpy
{  
   time_t fecha;
   struct tm *tmPtr;
   char nombre[60];
   FILE *fp;

   fecha = time(NULL);
   tmPtr = localtime(&fecha);
   	
   //Se abre archivo de escritura
   strftime(nombre, 60, "Tele_%d_%m_%y.txt", tmPtr);
   fp=fopen(nombre,"a");
   
   if(titulo == 0)
   {
   	fprintf(fp,"****************************TELEMETRIA*********************\n");
   	strftime(nombre, 60, "Fecha: %d/%m/%y", tmPtr);
   	fprintf(fp,"%s\n",nombre);
   	strftime(nombre, 60, "Hora: %H:%M:%S", tmPtr);
	fprintf(fp,"%s\n",nombre);

   //fp=fopen("/mnt/mmc/Telemetria.txt","a");		printf("ALT DES: %f --- ALT REAL: %f \n",laser_deseado,dist_laser);
   fprintf(fp,"UAV_timer,Pitch,Roll,Yaw,Heading,Alt_Bar,P,Q,R,Ax,Ay,Az,Tº,V_Pitot,Aleron,Elevador,Throttle,Timón,Lat,Lon,Alt,V_GPS,Rumbo_GPS,Rumbo_viento, int_viento, Modo de Operación,Distancia Laser\n");
   titulo = 1;
   }
   else{
  // fprintf(fp,"%f,",lsensor->v_sensor[muestra].tiempo);	

   fprintf(fp,"%f,",lsensor->v_sensor[muestra].ang_giro[0]); //Pitch
   fprintf(fp,"%f,",lsensor->v_sensor[muestra].ang_giro[1]); //Roll
   fprintf(fp,"%f,",lsensor->v_sensor[muestra].ang_giro[2]); //Yaw
   //fprintf(fp,"%f,",lsensor->v_sensor[muestra].rumbo); //Rumbo
   fprintf(fp,"%f,",lsensor->v_sensor[muestra].alt_bar); //Altura Barometrica

   fprintf(fp,"%f,",lsensor->v_sensor[muestra].v_giro[0]); //Velocidad de giro P en torno a X
   fprintf(fp,"%f,",lsensor->v_sensor[muestra].v_giro[1]); //Velocidad de giro Q en torno a Y
   fprintf(fp,"%f,",lsensor->v_sensor[muestra].v_giro[2]); //Velocidad de giro R en torno a Z

   fprintf(fp,"%f,",lsensor->v_sensor[muestra].a_intr[0]); //Aceleración Intrinseca X
   fprintf(fp,"%f,",lsensor->v_sensor[muestra].a_intr[1]); //Aceleración Intrinseca Y
   fprintf(fp,"%f,",lsensor->v_sensor[muestra].a_intr[2]); //Aceleración Intrinseca Z

//   fprintf(fp,"%f,",lsensor->v_sensor[muestra].a_giro);
   //fprintf(fp,"%f,",lsensor->v_sensor[muestra].t_aire); 

   fprintf(fp,"%f,",lsensor->v_sensor[muestra].v_pitot);

   fprintf(fp,"%f,",lsensor->v_sensor[muestra].aleron);
   fprintf(fp,"%f,",lsensor->v_sensor[muestra].elevador);
   fprintf(fp,"%f,",lsensor->v_sensor[muestra].throttle);
   fprintf(fp,"%f,",lsensor->v_sensor[muestra].timon);
   //fprintf(fp,"%f,",lsensor->v_sensor[muestra].lat_gps);
   //fprintf(fp,"%f,",lsensor->v_sensor[muestra].lon_gps);
   fprintf(fp,"%f,",lsensor->v_sensor[muestra].alt_gps);
   fprintf(fp,"%f,",lsensor->v_sensor[muestra].v_gps);
   //fprintf(fp,"%f,",lsensor->v_sensor[muestra].rumbo_gps);
   //fprintf(fp,"%f,",lsensor->v_sensor[muestra].rumbo_viento);
   //fprintf(fp,"%f,",lsensor->v_sensor[muestra].int_viento);
   //fprintf(fp,"%d,",uav_mode);
   //fprintf(fp,"%f\n",dist_laser);
}
   fclose(fp);
   return;
}



// This is a function that gets called every time a packet from the autopilot is received
void ReceivePacket(CCommPacket *NewPkt, PacketType Type)
{
	//Casting variables
	CStdTelemPacket *StdTelemPacket;


	if(NewPkt == NULL) return;

	switch(Type)
	{
	
	case STD_TELEMETRY_PACKET:

		StdTelemPacket = reinterpret_cast<CStdTelemPacket*>(NewPkt);

		//lsensor->v_sensor[muestra].tiempo = StdTelemPacket->GetAirborneTimer();
		
		lsensor->v_sensor[muestra].ang_giro[0]=	StdTelemPacket->GetPitch()*57.3;//angulo de cabeceo (variable longitudinal)

		pitch_actual = lsensor->v_sensor[muestra].ang_giro[0];

		lsensor->v_sensor[muestra].ang_giro[1] = StdTelemPacket->GetRoll()*57.3;//angulo de alabeo (variable lateral)
		lsensor->v_sensor[muestra].ang_giro[2] = StdTelemPacket->GetHeading()*57.3;//angulo de guinada yaw=heading (variable lateral)


		lsensor->v_sensor[muestra].alt_bar = StdTelemPacket->GetAltitude();//altura (vaiable longitudinal)
		
		alt_bar_gums = StdTelemPacket->GetAltitude();
		
		lsensor->v_sensor[muestra].rumbo = StdTelemPacket->GetHeading();

		lsensor->v_sensor[muestra].v_giro[0] = StdTelemPacket->GetRoll();//velocidad angular de cabeceo (variable longitudinal)
		lsensor->v_sensor[muestra].v_giro[1] = StdTelemPacket->GetPitch();//veocidad angular de alabeo (variable lateral)
		lsensor->v_sensor[muestra].v_giro[2] = StdTelemPacket->GetHeading();//velocidad angular de guinada(variable lateral)
	//	lsensor->v_sensor[muestra].aleron = StdTelemPacket->GetServo(0);//deflexion de aleron (variable lateral)
	
     //	lsensor->v_sensor[muestra].elevador = StdTelemPacket->GetServo(1);//deflexion del elevador (variable longitudinal)
	//	lsensor->v_sensor[muestra].throttle = StdTelemPacket->GetServo(2);//potencia de turbina (vaiable longitudinal)
	//	lsensor->v_sensor[muestra].timon = StdTelemPacket->GetServo(3);

		lsensor->v_sensor[muestra].v_pitot = StdTelemPacket->GetVelocity();

		//uav_mode = StdTelemPacket->GetUAVMode();
		
		
		escribir();

	break;



	}
}



/////kalman
mat predictKalman(mat (&x),mat (&F), mat (&B), mat (&P), mat (&Q)){
 
     x=F*x;//+B*0;
 
     P=(F*P)*(F.t())+Q;
 
     return x;
     }

void updateKalman(mat z,mat H, mat (&x), mat R, mat (&P)){ 
  
    mat S=R+ H*(P*(H.t()));
     
     
     mat K=(P*(H.t()))*inv(S);
     mat I;
     mat y;
     I.eye(5,5);
     
if(x.n_rows==1 || x.n_cols==1){//primera vez que se mide
     y=z;
     x=K*y;
     
     }
     else{
     y=z-(H*x);
     x=x+(K*y);     
          }
     
     P=((I-(K*H))*P)*((I-(K*H)).t())+ ((K*R)*(K.t()));

}




//Datos Lateral globales####
long double VelY=7.35*pow(10,-21);//Velocidad en el eje Y
long double VelAlabeo=8.19*pow(10,-11);//Velocidad angular de alabeo
long double VelGuinada=2.5*pow(10,-11);//Velocidad angular de guiñada
long double AnAlabeo=2.33*pow(10,-11);//Angulo de alabeo
long double RefAnGuinada=-2.45*pow(10,-16);//Angulo de guiñada
long double Aleron=0;//Deflexion del aleron
long double ErrorAcumLateral=0;//se debe guardar el error generado en cada iteracion
int IteracionLateral=0;//variable auxiliar opara sabr si es la primera vez que se aplica kalman
int nLat=5;//tamaño de la matriz de transicion(A o F)
int mLat=5;//Tamaño de la matriz de observacion (H)

mat FLat(5,5), BLat(5,1), HLat(1,5), QLat(5,5), RLat(1,1),PLat,xLat(nLat,1),KpLat(1,6),KiLat(1,1),lateralFiltrado;

void declararGlobalesLaterales(mat (&fLat),mat (&bLat),mat (&hLat),mat (&qLat),mat (&rLat),mat (&pLat),mat (&xLat)){
 
 //ganancias
 
 KpLat <<-0.263914681687843<<0.005870146814293<< -0.451118365306587<< -0.076515237405791<< -0.242917656036291<< 0.003148440985141<< endr;
 KiLat <<0.003148440985141<<endr;
 
 //matrices
fLat << -14.929 << 0 << -1 << 0.194 <<0 <<endr
     <<-3920.814 <<-15.634 << 1.282 << 0  <<0 <<endr
     <<-36.818<< -0.352 <<-0.141 << 0 <<0 <<endr
     <<0 << 1 << 0 << 0<< 0 <<endr
     <<0 << 0<< 0.696 <<0 << 0 <<endr;
     
     
bLat << 0<<endr
     <<27.42 <<endr
     <<-0.358<<endr
     <<0<<endr
      <<0 <<endr;

hLat <<0<<0<<0<<0<<1<<endr;

qLat << 0.0005<<0<<0<<0<<0 <<endr
     <<0<<0.01<<0<<0<<0 <<endr
     <<0<<0<<0.01<<0<<0<<endr
     <<0<<0<<0<<0.01<<0<<endr
      <<0<<0<<0<<0<<0.5<<endr;

rLat<<10<<endr;

pLat.eye(nLat,nLat);
xLat.zeros(); 
}


long double CaminoALateral(long double Ref, mat Sensado){//se lleva a cabo el camino sperior del modelo (Obtencion de error, multiplicar por Ki, aplicar integrador)


	long double resultado= (Ref-Sensado[5]);//error
	
	ErrorAcumLateral=(ErrorAcumLateral+resultado);//se debe gguardar el error acumulado
	resultado=as_scalar(resultado*KiLat);//aplicacion de ganancia
	resultado=resultado*Ts*ErrorAcumLateral;//integrador
	
	return resultado;}


long double CaminoBlateral(mat Sensado){
     
     if(IteracionLateral==0){//en caso de que sea la primera aplicacion de filtro de kalman
     for(int i=0;i<Sensado.n_rows; i++){
     lateralFiltrado=(HLat*predictKalman(xLat, FLat, BLat,PLat, QLat));
     updateKalman(Sensado.row(i), HLat,  xLat, RLat,  PLat);
     
     }IteracionLateral=1;
     }
     
     else{
     lateralFiltrado=(HLat*predictKalman(xLat, FLat, BLat,PLat, QLat));
     updateKalman(Sensado, HLat,  xLat, RLat,  PLat);
  }
  
	long double resultado=as_scalar(KpLat*(lateralFiltrado.t()));
	
     return resultado;
}

long double ControlLateral(long double Ref,mat Sensado){
	return (CaminoALateral(Ref,Sensado)+CaminoBlateral(Sensado));}
     

//Datos Longitudinal referencia####

long double RefVelX=9.13;//Velocidad en el eje X. Velocidad de referencia
long double VelZ=0.0573;//Velocidad en el eje Z
long double VelCabeceo=6.16*pow(10,-9);//Velocidad angular de cabeceo
long double AnCabeceo=0.00628;//Angulo de Cabeceo
long double Elevador=-0.699;//Deflexion del elevador
long double Turbina=-0.0288;//Potencia de la turbina
long double RefAltura=50;//Alturade referencia
long double ErrorAcumAltura=0;
long double ErrorAcumVelocidad=0;
int IteracionLongitudinal=0;//variable auxiliar opara sabr si es la primera vez que se aplica kalman     
     
mat FLong(5,5), BLong(5,2), HLongVelx(1,5), HLongAltura(1,5), QLong(5,5), RLong(1,1),PLongVelx,xLongVelx(nLat,1),PLongAltura,xLongAltura(nLat,1),KpLong(2,7),KpLongVelX(1,7),KpLongAltura(1,7),KiLong(2,2),longitudinalVelocidadFiltrado,longitudinalAlturaFiltrado;     
     
void declararGlobalesLongitudinales(mat (&flong),mat (&blong),mat (&hLongVelx),mat (&hLongAltura),mat (&qLong),mat (&rLong),mat (&pLongVelx),mat (&pLongAltura),mat (&xLongVelX),mat (&xLongAltura)){
 
 //ganancias
 
 KpLong  <<-0.132398284509918<<0.583870955844247<<-0.406968672517938<<-2.255809614055181<<0.354610418201763<<-0.001135192371066<<-0.043876385934576<<endr
         <<-0.105250870196647<<-0.009623922938968<<0.010547245467119<<0.910413754065392<<0.107176129418206<<0.022315669280681<< -0.002836336256071<<endr;
 
 KpLongVelX <<-0.132398284509918<<0.583870955844247<<-0.406968672517938<<-2.255809614055181<<0.354610418201763<<-0.001135192371066<<-0.043876385934576<<endr;
            
 KpLongAltura <<-0.105250870196647<<-0.009623922938968<<0.010547245467119<<0.910413754065392<<0.107176129418206<<0.022315669280681<< -0.002836336256071<<endr;
 
 KiLong <<-0.001135192371066 <<-0.043876385934576<<endr
        <<0.022315669280681<<  -0.002836336256071<<endr;
 
 //matrices
flong <<0.068<<-8.917<<8.245<<-9.779<<0<<endr
	<<-0.979<<-185.221<<-0.051<<-0.061<<0<<endr
	<<1.686<< -268.510<<-3.814<< 0<<0<<endr
	<<0<<0<<1<<0<<0<<endr
	<<-0.0062<<0.999<<0<<-9.127<<0<<endr;
     
     
     
blong <<-2.552<<-0.283<<endr
	<<-2.312<<0<<endr
	<<-24.53<<0<<endr
	<<0<<0<<endr
	<<0<<0<<endr;

hLongVelx <<1<<0<<0<<0<<0<<endr;
hLongAltura <<0<<0<<0<<0<<1<<endr; 

qLong <<0.05<<0<<0<<0<<0<<endr
     <<0<<0.01<<0<<0<<0<<endr
     <<0<<0<<0.01<<0<<0<<endr
     <<0<<0<<0<<0.01<<0<<endr
     <<0<<0<<0<<0<<2.5<<endr;

rLong<<20<<endr;
pLongVelx.eye(5,5);
pLongAltura.eye(5,5);
xLongVelX.zeros();
xLongAltura.zeros();

}

mat CaminoALongitudinal(long double RefVelocidadx , long double RefH, mat Sensado){
	
	mat resultado(1,2);
     resultado << RefVelocidadx-Sensado[5]<< RefH-Sensado[6]<<endr;//error
	ErrorAcumVelocidad=ErrorAcumVelocidad+resultado[0];//se debe guardar el error acumulado
	ErrorAcumAltura=ErrorAcumAltura+resultado[1];//se debe guardar el error acumulado
	resultado=resultado*(KiLong);//aplicacion de ganancia
	resultado[0]=resultado[0]*Ts*ErrorAcumVelocidad;//integrador
	resultado[1]=resultado[1]*Ts*ErrorAcumAltura;
	return resultado;}


	
mat CaminoBLongitudinal(mat Sensado){
	
     if(IteracionLongitudinal==0){//en caso de que sea la primera aplicacion de filtro de kalman
     for(int i=0;i<Sensado.n_rows; i++){
     longitudinalVelocidadFiltrado=(HLongVelx*predictKalman(xLongVelx, FLong, BLong,PLongVelx, QLong));
     longitudinalAlturaFiltrado=(HLongAltura*predictKalman(xLongAltura, FLong, BLong,PLongAltura, QLong));
     updateKalman(Sensado.row(i), HLongVelx,  xLongVelx, RLong,  PLongVelx);
     updateKalman(Sensado.row(i), HLongAltura,  xLongAltura, RLong,  PLongAltura);
     
     }IteracionLongitudinal=1;
     }
     
     else{
          
     longitudinalVelocidadFiltrado=(HLongVelx*predictKalman(xLongVelx, FLong, BLong,PLongVelx, QLong));
     longitudinalAlturaFiltrado=(HLongAltura*predictKalman(xLongAltura, FLong, BLong,PLongAltura, QLong));
     updateKalman(Sensado, HLongVelx,  xLongVelx, RLong,  PLongVelx);
     updateKalman(Sensado, HLongAltura,  xLongAltura, RLong,  PLongAltura);
  }
     
     
     mat Filtrado =join_cols(longitudinalVelocidadFiltrado,longitudinalAlturaFiltrado);
     
	mat resultado=KpLong*(Filtrado.t());
	return resultado;}



mat ControlLongitudinal(double long RefVelocidadx,double long RefH,mat Sensado){
	mat CaminoA=CaminoALongitudinal(RefVelocidadx,RefH,Sensado);
	mat CaminoB=CaminoBLongitudinal(Sensado);
	mat resultado(1,2);
     resultado[0]=CaminoA[0]+CaminoB[0];
     resultado[1]=CaminoA[1]+CaminoB[1];
	return resultado;}




int main(int argc, char** argv)
  {
 
 
 
 lsensor = (t_lsensor *)malloc(sizeof(t_lsensor));
	//lcalculo = (t_lcalculo *)malloc(sizeof(t_lcalculo));

	struct timeval Timeout;
	
	int maxfd,res, resg,fdg;
	char num[7];
	char d0,d1,d2,d3,d4,d5,d6,d7,d8;
	float numf;
	//int pitch_deseado = 0; //
	fd_set readfs;

	float pitch_des, aux_pitch, Kp, roll_ant, pitch_ant;
	int roll_cmd,pitch_cmd, max_pitch, min_pitch, mod_des;

	muestra = 0;
	counter1 = 0;
	counter2 = 0;
	counter3 = 0;
	
	titulo = 0;
	
	fdg = abre_puerto_gums();

	//Register a callback for the packets
	
	g_CommSerial.RegisterCallBack(ReceivePacket, ALL_PACKET);
	printf("funciono el packete\n");
/*
	//Open the serial port on the gumstix
	int ser_kestrel = g_CommSerial.Open("/dev/ttyS2");
	if(ser_kestrel==-1) return -1;

	//Creación de paquete de datos para recuperación de datos de telemetría Standard
	CWriteRawPacket ReqStdTelemPkt(STD_TELEMETRY_REQUEST_PACKET, 1032);

	
	//Creación de paquete de datos para recuperación de datos de Telemetria de GPS
	CWriteRawPacket ReqGPSTelemPkt(GPS_TELEMETRY_REQUEST_PACKET, 1032);
	//Creación de paquete de datos para recepción de datos de sensores
	CWriteRawPacket ReqSensorTelemPkt(REQUEST_SENSOR_PACKET, 1032);
	ReqSensorTelemPkt.AppendUnsignedChar(0); //Comando de envío de datos calibrados de sensores, Gumstix->Kestrel
	ReqSensorTelemPkt.SetPacketIDNum(counter1++); //Incremento en el Número de Identificador de Byte //dentro del paquete de Sensores.	

 
 */
 
 
 
 
//declararGlobalesLaterales(FLat,BLat,HLat,QLat,RLat,PLat,xLat);
//declararGlobalesLongitudinales(FLong, BLong, HLongVelx, HLongAltura, QLong, RLong,PLongVelx,PLongAltura,xLongVelx,xLongAltura);
/////////////kalman longitudinal






/////////////////Kalman Lateral

//mat lateralSensado;
//lateralSensado<<VelY<<VelAlabeo<<VelGuinada<<AnAlabeo<<Aleron<< RefAnGuinada<<endr
//<<(VelY-0.02)<<(VelAlabeo-0.002)<<(VelGuinada-0.032)<<(AnAlabeo-0.022)<<(Aleron-0.0012)<< (RefAnGuinada-0.2)<<endr;
  
//cout << "e= " << CaminoBlateral(lateralSensado)<< endl;
//lateralSensado <<(VelY-0.7)<<(VelAlabeo-0.7)<<(VelGuinada-0.7)<<(AnAlabeo-0.7)<<(Aleron-0.7)<< (RefAnGuinada-0.7)<<endr;
//cout << "e= " << CaminoBlateral(lateralSensado)<< endl;
//lateralSensado <<(VelY-0.7)<<(VelAlabeo-0.7)<<(VelGuinada-0.7)<<(AnAlabeo-0.7)<<(Aleron-0.7)<< (RefAnGuinada-0.7)<<endr;
//cout << "e= " << CaminoBlateral(lateralSensado)<< endl;
  
////////////////


    return 0;
  }
