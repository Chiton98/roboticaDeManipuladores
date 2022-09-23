import numpy as np

#Funcion para generar cada trayectoria en espacio articular
def generarTrayectoriaPolinomica(cIValores,t,tipo = "c"):
    """
        Entradas:
            cIValores (np.array) -> Vector columna de 2*n x 1(cubicas) o 3*n x 1 (quinticas) de las 
                                    condiciones iniciales de la articulacion
            t  (np.array) ->        Arreglo de tiempos
            
            tipo (str)  -> Tipo de trayectoria a generar, cúbica o quíntica
        Salidas:
            Q   -> Vector de n x elementos intervalo de la posicion de las articulaciones
            Qd  -> Vector de n x elementos intervalo de la velocidad de las articulaciones
            Qdd -> Vector de n x elementos intervalo de la aceleracion de las articulaciones
    """
    #Introducir los vectores iniciales
    t0 = t[0] #Primer elemento
    tf = t[-1] #Ultimo elemento

    if tipo == "c":
        #Trayectoria cubica

        #Matriz
        M = np.array([ [1,     t0,     t0*t0,     t0*t0*t0],
                        [0,     1,      2*t0,      3*t0*t0],
                        [1,     tf,     tf*tf,     tf*tf*tf],
                        [0,     1,      2*tf,      3*tf*tf]] )

        #Obtener los coeficientes 
        coeff = np.matmul( np.linalg.inv(M) , cIValores)
        
        print("Los coeficientes son:",coeff)

        #Definir posicion velocidad y aceleracion
        q = coeff[0][0] + coeff[1][0]*t + coeff[2][0]*t**2 + coeff[3][0]*t**3
        qd = coeff[1][0] + 2*coeff[2][0] * t + 3*coeff[3][0]*t**2
        qdd = 2*coeff[2][0] + 6*coeff[3][0]*t


        #Regresar los valores
        return q,qd,qdd

    
    elif tipo == "q":
        #Trayectoria quintica
        M = np.array([      [1,     t0,     pow(t0,2),      pow(t0,3),      pow(t0,4),      pow(t0,5)    ],
                            [0,     1,      2*t0,           3*pow(t0,2),    4*pow(t0,3),    5*pow(t0,4)  ],
                            [0,     0,      2,              6*t0,           12*pow(t0,2),   20*pow(t0,3) ],
                            [1,     tf,     pow(tf,2),      pow(tf,3),      pow(tf,4),      pow(tf,5)    ],
                            [0,     1,      2*tf,           3*pow(tf,2),    4*pow(tf,3),    5*pow(tf,4)  ],
                            [0,     0,      2,              6*tf,           12*pow(tf,2),   20*pow(tf,3) ],
                     ])

        #Obtener los coeficientes 
        coeff = np.matmul( np.linalg.inv(M) , cIValores)

        #Definir la posicion, velocidad y aceleracion



    
    else:
        print("NO hay esa opcion")

#Funcion para generar la trayectoria articular con puntos via
def generarTrayectoriaArticularPuntosVia(arregloCI,T,t,n = 4):
    """
    Función para generar la trayectoria articular mediante 
    puntos via.
    Sólo tendremos control de la posición.
    La velocidad y aceleración sólo sabemos que serán continuas.

    Entradas:
        arregloCI (np.array nx1) -> Arreglo que contiene los puntos via
        T(np.array n,) -> Arreglo que contiene los valores del tiempo para cada punto via
        t (n.array n,) -> Arreglo que contiene el arreglo de tiempos

    Salidas:
        Pos(np.array) -> Posicion del parametro de Euler
        Vel
        Acel

    """

    #Matriz de coeficientes
    M = np.array([[1,T[0],T[0]**2,T[0]**3,T[0]**4],
                  [1,T[1],T[1]**2,T[1]**3,T[1]**4],
                  [1,T[2],T[2]**2,T[2]**3,T[2]**4],
                  [1,T[3],T[3]**2,T[3]**3,T[3]**4],
                  [1,T[4],T[4]**2,T[4]**3,T[4]**4] ])

    #Obtener los coeficientes de los polinomios
    a = np.matmul(np.linalg.inv(M),arregloCI)

    #Generar las trayectorias
    Pos = a[0,0] + a[1,0]*t + a[2,0]*t**2+ a[3,0]*t**3+a[4,0]*t**4


    return Pos

#Funcion para generar las trayectorias en espacio cartesiano
def generarTrayectoriaEspacioCartesiano(objetoRobot,Q,Qd,Qdd):
    P0e =  objetoRobot.obtenerPosicionEfectorFinal(Q)
    V0e =  objetoRobot.obtenerVelocidadEfectorFinal(Q,Qd)
    A0e =  objetoRobot.obtenerAceleracionEfectorFinal(Q,Qd,Qdd)

    return P0e,V0e,A0e

#Funcion para generar todas las trayectorias en espacio articular

def generarTodasTrayectoriaArticular(n,matrizCI,vTiempo,tipo = "c"):
    """
    Funcion que genera todas las trayectorias articulares

    Entradas:
        n -> Grados de libertad del robot
        matrizCI -> Matriz que contiene las condiciones iniciales de todas las articulaciones
                    por la columna i especifica las condiciones para la articulacion i
        vTiempo(np.array) Contiene en forma de vector renglon los tiempos a evaluar la trayectoria.

        Y los demás que son propios de la trayectoria individual
    
    Salidas:
        Q,Qd,Qdd -> Vectores de n x tamaño del vector tiempo que contiene todas las trayectorias
                    de posicion, velocidad y aceleración angular.
    """
    
    #Lista ara almacenar las trayectorias
    Q = np.empty((n,len(vTiempo)))
    Qd = np.empty((n,len(vTiempo))) 
    Qdd = np.empty((n,len(vTiempo)))

    for i in range(n):
        #Obtener la trayectoria individual
        q,qd,qdd = generarTrayectoriaPolinomica(matrizCI[:,i].reshape(4,1),vTiempo,tipo)

        #Almacenar las trayectorias
        Q[i,:] = q
        Qd[i,:] = qd
        Qdd[i,:] = qdd
    
    #Regresar las trayectorias
    return Q,Qd,Qdd
