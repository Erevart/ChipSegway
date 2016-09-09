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
	private LinkedList<Double> bufferdata;


	
	// Mutex - Lock
	private Lock lock_log;
	
	// Flag
	private boolean isClient = false;
	private boolean isServer = false;
	
	// 
	private Object data = null;
	
	public DataLoggerWifi(){

		/* Atributos de permiso y acceso */
		lock_log = new ReentrantLock();
		
		/* Inicialización de atributos */
		setServerStatus(true);
		
		buffertype_data= new LinkedList<Integer>();
		bufferdata= new LinkedList<Double>();
		
	}
	
	private void sendData(int type_data, double data ){
		
			
		if (!getClientStatus())
			return;
				       
		try {
			transmit_data.writeChar(type_data);
			transmit_data.writeDouble(data);
		} catch (IOException e) {
			if (Segway.WIFILOGDB)
				System.out.println("No conexión con el servidor");
			setClientStatus(false);
			return;
		}
		
	}
	
	private void sendDouble(double data ){
		
		
		if (!getClientStatus())
			return;
		
					       
		try {
			transmit_data.writeDouble(data);
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
				System.out.println("No conexión con el servidor");
			setClientStatus(false);
			return;
		}
		
	}
	
	private char receivedChar(){
		
		if (!getClientStatus())
			return 0;
		
		long timestamp = 0;
		char datain[] = new char[1];
		
		sendChar('@');
		
		try {
			/*
			// Si pasados 10s no se recibe respuesta se lanza un error.
			timestamp = System.currentTimeMillis();
			while (!received_data.ready()){
				if ((System.currentTimeMillis() - timestamp) > 5000)
					 throw new IOException();
			}
			*/
			sendChar('@');
			received_data.read(datain);
		} catch (IOException e) {
			if (Segway.WIFILOGDB)
				System.out.println("No hay respuesta del servidor");
			setClientStatus(false);
			return 0;
		}
		
		return datain[0];
		
	}
	
	public void close(){
		setServerStatus(false);
	}
	
	
	private void setClientStatus(boolean status){
		
		lock_log.lock();
		isClient = status;
		lock_log.unlock();
		
	}
	
	private boolean getClientStatus(){
		
		lock_log.lock();
		boolean _isClient = isClient;
		lock_log.unlock();
		
		return _isClient;
		
	}
	
	
	private void setServerStatus(boolean status){
		
		lock_log.lock();
		isServer = status;
		lock_log.unlock();
		
	}
	
	private boolean getServerStatus(){
		
		lock_log.lock();
		boolean _isServer = isServer;
		lock_log.unlock();
		
		return _isServer;
		
	}
	
	public void setDataLog(char type_data, double data){	
		
		if (!getClientStatus())
			return;
		
		lock_log.lock();
		buffertype_data.add(new Integer (type_data));
		bufferdata.add(new Double (data));
		
		datacounter++;
		lock_log.unlock();
		
		System.out.println(datacounter);
	}
	
	@Override
	public void run(){
		
		// Configurar prioridad del sistema
		Thread.currentThread().setPriority(Thread.MIN_PRIORITY);
		
		int _datacounter = 0;
		int _type_data; 
		double _data;
		double i = 0;
		
		try {
			serversocket = new ServerSocket(PORT);
			do {
				socket = serversocket.accept();
				transmit_data = new DataOutputStream(socket.getOutputStream());
				received_data = new BufferedReader(new InputStreamReader(socket.getInputStream()));
				if (Segway.WIFILOGDB)
					 System.out.println("Conexion establecida");	 
				setClientStatus(true);
				while(getClientStatus()){
					
					lock_log.lock();
					_datacounter = datacounter;
					lock_log.unlock();
					
					if (receivedChar() == '$' && _datacounter != 0){
					//	sendDouble(_datacounter);
					//	for (int i = 0; i < _datacounter; i++){
							lock_log.lock();
							_type_data = buffertype_data.removeFirst();
							_data = bufferdata.removeFirst();
							lock_log.unlock();
							
							sendData((char)_type_data,_data);
					//	}
						lock_log.lock();
					//	datacounter -= _datacounter;
						datacounter-- ;
						lock_log.unlock();
					}		
					
					try {Thread.sleep(30);} catch (InterruptedException e) {e.printStackTrace();}
				 }
				if (Segway.WIFILOGDB)
					 System.out.println("Cliente desconectado");
				 transmit_data.close();
				 received_data.close();
				 socket.close();
				 
			 }while(getServerStatus());			 
			 serversocket.close();
			
		 	} catch (IOException e) { if (Segway.WIFILOGDB) System.out.print("Error, no se creo el servidor. \n");}
			 

	}
	

}
