package segway;

import java.io.*;
import java.net.*;
import java.util.LinkedList;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import lejos.hardware.BrickFinder;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.TextLCD;

public class DataLoggerWifi extends Thread  {
	
	private final int PORT = 8080;
	
	private ServerSocket serversocket = null;
	private Socket socket = null;
	
	// Variables
	private int datacounter = 0; // Indica el numero de elementos del buffer de datos.
	
	// Buffer
	private BufferedReader received_data = null;
	private DataOutputStream transmit_data = null;
	private LinkedList<Integer> buffertype_data;
	private LinkedList<Float> bufferdata;
	
	// Mutex - Lock
	private Lock lock_logdata;
	private Lock lock_logstatus;
	
	// Flag
	private boolean flag_isClient = false;
	private boolean flag_isServer = false;
	
	
	public DataLoggerWifi(){

		/* Atributos de permiso y acceso */
		lock_logdata = new ReentrantLock();
		lock_logstatus = new ReentrantLock();
		
		/* Inicialización de atributos */
		setServerStatus(true);
		
		buffertype_data= new LinkedList<Integer>();
		bufferdata= new LinkedList<Float>();
		
	}
	
	private void sendData(char type_data, float data ){
				
		if (!getClientStatus())
			return;
				       
		try {
			transmit_data.writeChar(type_data);
			transmit_data.writeFloat(data);
		} catch (IOException e) {
			if (Segway.WIFILOGDB)
				System.out.println("No conexión con el servidor");
			setClientStatus(false);
			return;
		}
		
	}
	
	private void sendfloat(float data ){
		
		
		if (!getClientStatus())
			return;
		
					       
		try {
			transmit_data.writeFloat(data);
		} catch (IOException e) {
			if (Segway.WIFILOGDB)
				System.out.println("No conexión con el servidor");
			setClientStatus(false);
			return;
		}
		
	}
	
	private void sendChar(char data ){
		
		
		if (!getClientStatus())
			return;
		
					       
		try {
			transmit_data.writeChar(data);
		} catch (IOException e) {
			if (Segway.WIFILOGDB)
				System.out.println("W:No conexión con el servidor");
			setClientStatus(false);
			return;
		}
		
	}
	
	private char receivedChar(){
		
		//sendChar('@');
		
		if (!getClientStatus())
			return 0;
		
		long time = 0;
		char datain[] = new char[1];
		
		try {
			// Si pasados 5s no se recibe respuesta se lanza un error.
			time = System.currentTimeMillis();
			while (!received_data.ready()){
				if ((System.currentTimeMillis() - time) > 5000)
					 throw new IOException();
			}
			received_data.read(datain);
		} catch (IOException e) {
			if (Segway.WIFILOGDB)
				System.out.println("R:No hay respuesta del servidor");
			setClientStatus(false);
			return 0;
		}
		
		return datain[0];
		
	}
	
	public void close(){
		setClientStatus(false);
		setServerStatus(false);
//		try {socket.close();} catch (IOException e) {}
	}
	
	
	private void setClientStatus(boolean status){
		
		lock_logstatus.lock();
		flag_isClient = status;
		lock_logstatus.unlock();
		
	}
	
	private boolean getClientStatus(){
		
		lock_logstatus.lock();
		boolean _flag_isClient = flag_isClient;
		lock_logstatus.unlock();
		
		return _flag_isClient;
		
	}
	
	
	private void setServerStatus(boolean status){
		
		lock_logstatus.lock();
		flag_isServer = status;
		lock_logstatus.unlock();
		
	}
	
	private boolean getServerStatus(){
		
		lock_logstatus.lock();
		boolean _flag_isServer = flag_isServer;
		lock_logstatus.unlock();
		
		return _flag_isServer;
		
	}
	
	public void setDataLog(char type_data, float data){	
		
		if (!getClientStatus())
			return;
		
		lock_logdata.lock();
			buffertype_data.add(new Integer (type_data));
			bufferdata.add(new Float (data));
			
			datacounter++;
		System.out.println(datacounter);
		lock_logdata.unlock();
	}
	
	@Override
	public void run(){
		
		// Configurar prioridad del sistema
		Thread.currentThread().setPriority(Thread.MIN_PRIORITY);
		
		int _datacounter = 0;
		int _type_data = 'A';
		float _data = 0f;
		
		try {
			serversocket = new ServerSocket(PORT);
			do {
				socket = serversocket.accept();
				transmit_data = new DataOutputStream(socket.getOutputStream());
				received_data = new BufferedReader(new InputStreamReader(socket.getInputStream()));
				if (Segway.WIFILOGDB)
					 System.out.println("Conexion establecida");	 
				setClientStatus(true);
				/*
				 * Loop 33 Hz 
				 */
				while(getClientStatus()){
					
					if (receivedChar() == '$'){ 
						lock_logdata.lock();
						_datacounter = datacounter;
						if (datacounter != 0){
							_type_data = buffertype_data.removeFirst();
							_data = bufferdata.removeFirst();
							datacounter-- ;
						}
						lock_logdata.unlock();
						if (_datacounter != 0)
							sendData((char)_type_data,_data);
						else 
							sendData((char)'!',0);
					}		
					
					try {Thread.sleep(30);} catch (InterruptedException e) {e.printStackTrace();}
				 }
				buffertype_data.clear();
				bufferdata.clear();
				datacounter = 0;
				if (Segway.WIFILOGDB)
					 System.out.println("Cliente desconectado");
				 transmit_data.close();
				 received_data.close();
				 if (!socket.isClosed())
					 socket.close();
				 
			 }while(getServerStatus());			 
			 serversocket.close();
			
		 	} catch (IOException e) { if (Segway.WIFILOGDB) System.out.print("Error, no se creo el servidor. \n");}
	}
	

}
