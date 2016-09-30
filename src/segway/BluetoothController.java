package segway;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import lejos.internal.ev3.EV3GraphicsLCD;
import lejos.remote.nxt.BTConnector;
import lejos.remote.nxt.NXTConnection;

public class BluetoothController{
	
	public static final byte ID_EV3 = (byte) 0xE3;
	public static final byte ID_BT = (byte) 0x3E;
	public static final byte CMD_SPEED = (byte) 0xF1;
	public static final byte CMD_STEERING = (byte) 0xF2;
	
	public static final byte[] ALIVE = {ID_BT,ID_EV3};
	
	private BTConnector BTsocket = null;
	private NXTConnection btclient = null;
	
	// Mutex - Lock
	private Lock lock_btcstatus;
	private Lock lock_obstacle;
	
	// Flag
	private boolean flag_isConnect = true;
	private boolean obstacle = false;
	
	
	public BluetoothController(){
		
		// Permite establecer una conexión Bluetooth
		System.out.close();
		BTsocket = new BTConnector();	
		
		/* Atributos de permiso y acceso */
		lock_btcstatus = new ReentrantLock();
		lock_obstacle = new ReentrantLock();
		
	}
	
	public void close (){
		setStatus(false);
		try {btclient.close();} catch (IOException e) {}
		BTsocket.cancel();
	}
	
	private void setStatus(boolean status){
		
		lock_btcstatus.lock();
		flag_isConnect = status;
		lock_btcstatus.unlock();
		
	}
	
	private boolean getStatus(){
		
		lock_btcstatus.lock();
		boolean _flag_isConnect = flag_isConnect;
		lock_btcstatus.unlock();
		
		return _flag_isConnect;
		
	}
	
	private void setSteering(float _steering){
		
		if (getobstacleidentified())
			return;
		
		Stabilizer.setSteering(_steering);
	}
	
	private void setSpeed(float _speed){
		
		if (getobstacleidentified())
			return;
		Stabilizer.setSpeed(_speed);
	}
	
	public void setobstacleidentified(boolean _obstacle){
		
		lock_obstacle.lock();
		obstacle = _obstacle;
		lock_obstacle.unlock();
	}
	
	public boolean getobstacleidentified(){
		
		boolean _obstacle;
		
		lock_obstacle.lock();
		_obstacle = obstacle;
		lock_obstacle.unlock();
		
		return _obstacle;
	}
	
	
	/**
	 * 
	 * Definición del hilo mediante interface de Runnable
	 *
	 */
	private class BluetoothControllerThread implements Runnable { 
	
		private BluetoothControllerThread() {
			super(); 
		}
	
		@Override
		public void run(){
			
			btclient = BTsocket.waitForConnection(0, NXTConnection.RAW);
			
			DataInputStream datareceived = new DataInputStream(btclient.openDataInputStream());
			DataOutputStream datatransmitted = new DataOutputStream(btclient.openDataOutputStream());
			
			byte id = 0;
			int num_data = 0;
			
			while (getStatus()){			
				try {
					id = (byte)datareceived.read();
					if (id == ID_EV3){			
						num_data = datareceived.available();
						while (num_data != 0){
							switch((byte)datareceived.read()){
							case CMD_SPEED:
								setSpeed((float)datareceived.readByte());
								break;
							case CMD_STEERING:
								setSteering((float)datareceived.readByte());
								break;
							}
							num_data--;
						}
						}else if (id == -1){
						//	BTsocket.close;	
						}
				} catch (IOException e1) {}
			}
			try {datareceived.close();} catch (IOException e) {}
			try {datatransmitted.close();} catch (IOException e) {}		
			//try {btclient.close();} catch (IOException e) {}
			BTsocket.cancel();
		
		}
		
	}
	
	/**
     * Invoca e inicia el Thread Stabilizer para estabilizar el robot o
	 * 			o lograr su desplazamiento sin perder el equilibrio.
	 * @param - none
	 * @return 	none
	 */
	public void start() {
		Thread BluetoothController = new Thread(new BluetoothControllerThread());
		BluetoothController.setPriority(6);
		BluetoothController.start();
		
	}
	
}
