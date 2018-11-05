#!/usr/bin/python
# -*- coding: iso-8859-15 -*-

import numpy as np
import time 
from scipy import signal


PACKET_LEN = 14
DATA_LEN = 7

A_X = 0
A_Y = 1
A_Z = 2
TMP = 3
G_X = 4
G_Y = 5
G_Z = 6



SSF_A_def = 16384 #[LSB/g]
SSF_G_def = 131  # [LSB / (º/s)]
PROM_SIZE_ACEL_def = 25
PROM_SIZE_GYRO_def = 3
A_err_def=3
A_err_temp_def=0.02
A_err_offset_def = 50e-3
G_err_def=3
G_err_offset_def = 20



def get_datos_estatico(ser, Ndatos = 10):
    
    lista_in = list()
    medicion = np.zeros((DATA_LEN))
    time_list = list()
    DELTA_t = 0.05
    

    # Pido un par de paquetes mas del reposo
    for i in range(Ndatos):

        ser.write(b'.') 
        data = ser.read(PACKET_LEN)


        np_data = np.array(list(data),dtype=np.uint8)

        act_read = np.zeros((DATA_LEN), dtype=np.int16)
        act_feed = np.zeros((DATA_LEN), dtype=np.float32)

        for i in range(DATA_LEN):

            act_read[i] = np_data[(i*2)+0]*(2**8) + np_data[(i*2)+1]
            if i == TMP:
                act_feed[i] = (act_read[i]/340.00)+36.53
            else:
                act_feed[i] = act_read[i]


            medicion[i] =  act_feed[i]


        lista_in.append(np.copy(medicion))
        time.sleep(DELTA_t)  
        
        
    medicion = np.array(lista_in)

    mean_time_sample = np.array(time_list).mean()
    
    return (medicion, mean_time_sample)

def get_datos(ser, orden, orden2 = 'a', time2orden = -1):
    
    lista_in = list()
    medicion = np.zeros((DATA_LEN))
    time_list = list()
    DELTA_t = 0.05

    count_to_speedup = 15/DELTA_t

    ser.write(str.encode(orden)) 
    
    while(1):

        t_ini = time.time()

        if time2orden > 0 :
            count_to_speedup = count_to_speedup - 1
            
        if count_to_speedup == 0:
            ser.write(str.encode(orden2)) 
        else:
            ser.write(b'.') 



        data = ser.read(PACKET_LEN)


        np_data = np.array(list(data),dtype=np.uint8)

        act_read = np.zeros((DATA_LEN), dtype=np.int16)
        act_feed = np.zeros((DATA_LEN), dtype=np.float32)

        suma = 0
        for i in range(DATA_LEN):

            act_read[i] = np_data[(i*2)+0]*(2**8) + np_data[(i*2)+1]
            suma = suma+act_read[i]
            if i == TMP:
                act_feed[i] = (act_read[i]/340.00)+36.53
            else:
                act_feed[i] = act_read[i]


            medicion[i] =  act_feed[i]


        if suma == 0:
            break

        lista_in.append(np.copy(medicion))
        time.sleep(DELTA_t)  
        time_list.append(time.time()-t_ini)

    # Pido un par de paquetes mas del reposo
    for i in range(5):

        ser.write(b'.') 
        data = ser.read(PACKET_LEN)


        np_data = np.array(list(data),dtype=np.uint8)

        act_read = np.zeros((DATA_LEN), dtype=np.int16)
        act_feed = np.zeros((DATA_LEN), dtype=np.float32)

        for i in range(DATA_LEN):

            act_read[i] = np_data[(i*2)+0]*(2**8) + np_data[(i*2)+1]
            suma = suma+act_read[i]
            if i == TMP:
                act_feed[i] = (act_read[i]/340.00)+36.53
            else:
                act_feed[i] = act_read[i]


            medicion[i] =  act_feed[i]


        lista_in.append(np.copy(medicion))
        time.sleep(DELTA_t)  
        
        
    medicion = np.array(lista_in)

    mean_time_sample = np.array(time_list).mean()
    
    return (medicion, mean_time_sample)



def signal0to360_HARDCODED(signal_in, eje):

    if eje == 0:
        # Es el eje X
        idx_max = np.argmax(signal_in)
        signal_in[idx_max:] = (signal_in.max() - signal_in[idx_max:])+signal_in.max()
    else:
        # Es el eje Y
        
        # Busco el primer pico
        idx_max = np.argmax(np.abs(signal_in[:int(len(signal_in)/2)]))
        # me fijo para donde va
        direccion = np.sign( signal_in[idx_max] )

        if direccion > 0: 
            # Corrijo el primer pico
            signal_in[idx_max:] = (signal_in[idx_max] - signal_in[idx_max:]) + signal_in[idx_max]
            # corrijo el segundo
            idx_max = np.argmax(signal_in)
            signal_in[idx_max:] = (signal_in.max() - signal_in[idx_max:])+signal_in.max()
            
        else:
            signal_in[:idx_max] = -signal_in[:idx_max]
            signal_in[idx_max:] = (signal_in[idx_max:] - signal_in[idx_max]) - signal_in[idx_max]
            # corrijo el segundo
            idx_max = np.argmax(signal_in)
            signal_in[idx_max:] = (signal_in.max() - signal_in[idx_max:])+signal_in.max()



    return signal_in  

def signal360to90_HARDCODED(signal_in, base_ang = 90):
    
        idx_sup = np.argwhere(signal_in > base_ang)

        signal_in[idx_sup] = (2*base_ang)-signal_in[idx_sup] 
        
        idx_sup = np.argwhere(signal_in < -base_ang)

        signal_in[idx_sup] = -(2*base_ang)-signal_in[idx_sup] 
        
        idx_sup = np.argwhere(signal_in > base_ang)

        signal_in[idx_sup] = (2*base_ang)-signal_in[idx_sup] 
        
        return signal_in


def acel2deg(signal_in):
    
    out_signal = np.copy(signal_in)
    out_signal[out_signal > 1]=  1
    out_signal[out_signal < -1]=  -1
    out_signal = np.arcsin(out_signal)
    out_signal = np.rad2deg(out_signal)
    
    return out_signal

def get_signals(medicion, 
                PROM_SIZE_ACEL = PROM_SIZE_ACEL_def, 
                PROM_SIZE_GYRO = PROM_SIZE_GYRO_def, 
                sample_time = -1, 
                ang_ini = 0,
                SSF_A = SSF_A_def, 
                SSF_G = SSF_G_def, 
                A_err = A_err_def, 
                A_err_temp = A_err_temp_def,
                A_err_offset = A_err_offset_def, 
                G_err = G_err_def, 
                G_err_offset = G_err_offset_def):
    
    num_samp , _ = medicion.shape
    
    out_signals = np.zeros((3,num_samp))
    out_std = np.zeros((3,num_samp))
    out_rel_err = np.zeros((3,num_samp))

    #sample_time = sample_time*1.05
    
    # Acelerometro X
    out_signals[0,:] = np.copy(medicion[:,A_X])
    out_signals[0,:] = signal.medfilt(out_signals[0,:], kernel_size=PROM_SIZE_ACEL)
    
    out_std[0,:] = np.sqrt( ((out_signals[0,:]/(SSF_A**2) )*SSF_A*(A_err + (A_err_temp*medicion[:,TMP]))/(100*np.sqrt(3)) )**2 + (A_err_offset/np.sqrt(3))**2 )

        
    out_signals[0,:] = out_signals[0,:]/SSF_A
    
    out_std[0,:] = acel2deg(out_std[0,:])
    out_rel_err[0,:] = np.divide(acel2deg(out_std[0,:]),acel2deg(out_signals[0,:]))

    out_signals[0,:] = acel2deg(out_signals[0,:])


    # Acelerometro Y
    out_signals[1,:] = np.copy(medicion[:,A_Y])
    out_signals[1,:] = signal.medfilt(out_signals[1,:], kernel_size=PROM_SIZE_ACEL)
    
    out_std[1,:] = np.sqrt( ((out_signals[1,:]/(SSF_A**2) )*SSF_A*(A_err + (A_err_temp*medicion[:,TMP]))/(100*np.sqrt(3)) )**2  + (A_err_offset/np.sqrt(3))**2 ) 

        
    out_signals[1,:] = out_signals[1,:]/SSF_A
    out_std[1,:] = acel2deg(out_std[1,:])
    out_rel_err[1,:] = np.divide(acel2deg(out_std[1,:]),acel2deg(out_signals[1,:]))

    out_signals[1,:] = acel2deg(out_signals[1,:])
    
    
    # Gyroscopo
    out_signals[2,:] = np.copy(medicion[:,G_Z])
    out_signals[2,:] = signal.medfilt(out_signals[2,:], kernel_size=PROM_SIZE_GYRO)
    
    dirreccion = np.sign(out_signals[2,:].mean())
    
    if sample_time > 0:
        out_std[2,:] = ( ( (sample_time*out_signals[2,:]/(SSF_G**2) )*SSF_G*(G_err)/(100*np.sqrt(3)) ) ** 2  + (sample_time*G_err_offset/np.sqrt(3))**2 )
        out_std[2,:] = out_std[2,:].cumsum()
        out_std[2,:] = np.sqrt(out_std[2,:])
        out_signals[2,:] = (sample_time*(out_signals[2,:]/SSF_G))
        out_signals[2,0] = out_signals[2,0] + ang_ini
        out_signals[2,:] = out_signals[2,:].cumsum()
        out_rel_err[2,:] = np.divide(out_std[2,:],out_signals[2,:])
        
        #if (out_signals[2,-1] < 0):
        #    out_signals[2,:] = -out_signals[2,:]
        #out_signals[2,:] = signal360to90_HARDCODED(out_signals[2,:])
    else:
        out_std[2,:] = ( ( (out_signals[2,:]/(SSF_G**2) )*SSF_G*(G_err)/(100*np.sqrt(3)) ) ** 2  + (G_err_offset/np.sqrt(3))**2 )
        out_std[2,:] = np.sqrt(out_std[2,:])
        out_signals[2,:] = (out_signals[2,:]/SSF_G)
        out_rel_err[2,:] = np.divide(out_std[2,:],out_signals[2,:])

    
    # PAra poder saber la direccion usamos la info del otro eje
    pos0_pos1 = np.logical_and(out_signals[0,:] > 0, out_signals[1,:] > 0)
    neg0_pos1 = np.logical_and(out_signals[0,:] < 0, out_signals[1,:] > 0)
    neg0_neg1 = np.logical_and(out_signals[0,:] > 0, out_signals[1,:] < 0)
    pos0_neg1 = np.logical_and(out_signals[0,:] < 0, out_signals[1,:] < 0)    

    
    out_signals[0,pos0_pos1] = out_signals[0,pos0_pos1] - 180 - 90
    out_signals[0,neg0_pos1] = out_signals[0,neg0_pos1] - 180 - 90
    out_signals[0,neg0_neg1] = -out_signals[0,neg0_neg1] - 90
    out_signals[0,pos0_neg1] = -out_signals[0,pos0_neg1] - 90
    
    
    out_signals[1,pos0_pos1] = -out_signals[1,pos0_pos1] - 180 
    out_signals[1,neg0_pos1] = out_signals[1,neg0_pos1] - 180 - 180
    out_signals[1,neg0_neg1] = -out_signals[1,neg0_neg1] - 180
    out_signals[1,pos0_neg1] = out_signals[1,pos0_neg1] 


    if dirreccion > 0:
        out_signals[0,:] = 360+out_signals[0,:]
        out_signals[1,:] = 360+out_signals[1,:]
    

    return out_signals, out_std, out_rel_err

    
    
    
    