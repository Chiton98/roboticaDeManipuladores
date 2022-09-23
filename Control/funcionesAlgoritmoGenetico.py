import numpy as np
import random


def convertirVectorKtoMatrix(K,n):
    #Crear arreglos vacios
    Kp = np.zeros([len(K)//2,len(K)//2])
    Kv = np.zeros([len(K)//2,len(K)//2])
    
    #Llenar la diagonal principal
    np.fill_diagonal(Kp,K[0:n])
    np.fill_diagonal(Kv,K[n:2*n])

        
    
    return Kp,Kv

#Funcion que permite obtener el par Calculado FIT
def parCalculadoFIT(objetoRobot,K,t,Q,Qd,Qdd,fit = True):


    ''' 
    Entradas : 
        R -> Es el objeto robot que contiene las matrices dinamicas
        K -> Es un np.array de 2x1 donde K[0] es kp y K[1] es kv
    
    '''
    #Grados de libertad
    n = objetoRobot.n
    
    #Tranformar el vector K, a las matrices Kp y Kv
    Kp,Kv = convertirVectorKtoMatrix(K,n)
    
    
    #SE SUPONE QUE EL ROBTO INICIAL DESDE LA POSICION HOME
    

    #Configuración de los valores
    #kp1 = K[0][0]
    #kv1 = K[1][0]
    #kp2 = K[2][0]
    #kv2 = K[3][0]
    
    #Generacion de arreglos de los valores actuales
    
    #Posicion actual
    Qa = np.empty([n,len(t)])

    #Velocidad actual
    Qda = np.empty([n,len(t)])

    #Aceleracion actual
    Qdda = np.empty([n,len(t)])

    #Torque actual
    Taua = np.empty([n,len(t)])

    #Tamaño de paso
    dt = 0.01
    
    #Condiciones iniciales para la posicion actual y la velocidad actual
    for i in range(n):
        Qa[i][0] = 0
        Qda[i][0] = 0

    #Arreglos para los erores
    Ep = np.empty([n,len(t)])
    Ev = np.empty([n,len(t)])
      
    #--------------------------Realizacion del par calculado-------------------------------
    
    #Iterar por todo el intervalo del tiempo
    for i in range(Q.shape[1]):
        
        #Comparar los valores deseados con los actuales y asignarlos al vector de error
        Ev[:,i] = Qd[:,i] -  Qda[:,i] #Velocidad deseada - Velocidad actual
        Ep[:,i] = Q[:,i] - Qa[:,i] #Posicion deseada - Posicion actual

        #Evaluar tau'
        tau_prime = Qdd[:,i].reshape(n,1) + np.matmul(Kv,Ev[:,i].reshape(n,1)) + np.matmul(Kp,Ep[:,i].reshape(n,1))
       
        #Almacenar la matriz de masas
        M = objetoRobot.getMatrizDeMasas(Qa[:,i])

        #Almacenar matriz de Cor y Cen
        V = objetoRobot.getMatrizCorCen(Qa[:,i],Qda[:,i])

        #Almacenar la matriz de términos centrifugos
        G = objetoRobot.getMatrizGravedad(Qa[:,i])

        #Multiplicar la matriz de masas por el vector tau'
        alfa_tau_prime = np.matmul(M,tau_prime)#alfa_trau_prime es de nx1

        #Evaluar beta
        beta = V + G #beta es de nx1

        #Evaluar el torque
        Taua[:,i] = (alfa_tau_prime + beta).reshape(n,)

        #Realizar la simulacion dinamica
        Qdda[:,i] = np.matmul(np.linalg.inv(M),(Taua[:,i].reshape(n,1) - V - G)).reshape(n,)

        try:  
            #Actualizar la velocidad
            Qda[:,i+1] = Qda[:,i] + Qdda[:,i]*dt  
            #Actualizar la posicion
            Qa[:,i+1] = Qa[:,i] + Qda[:,i]*dt + 0.5*Qdd[:,i]*dt*dt
        except IndexError:
            pass
        
    if fit == True:
        #Obtener el error de las articulaciones
        EP = 0
        EV = 0
        for i in range(n): #Iterar por todos los grados de libertad y sumar sus errores
            EP += np.sum(np.square(Ep[i]))
            EV += np.sum(np.square(Ev[i]))
            
        #Regresar el promedio del error de las articulaciones 
        return (EP+EV)/2
    
    elif fit==False:
        return Qa,Qda,Qdda,Taua,Ep,Ev

#Funcion de apoyo para determinar si al menos un elemento del vector Noisy es negativo
def esNegativoNoisy(Noisy):
    for i in Noisy:
        if i < 0:
            return True
        

#1.-Inicializacion
def inicializacion(objetoRobot,objetoGenetico,valueXMin,valueXMax):
    '''
        Entradas:
            objetoRobot -> Robot que contiene los gdl
            objetoGenetico -> Objeto genetico que contiene los parámetros para el algoritmo genetico
            valueXMin -> Valor minimo que podrán tomar las ganancias
            valueXMax -> Valor máximo que podrán tomar las ganancias
        
        Salidas:
            P -> Población inicial con NP individuos de la generación actual
    '''
    print("Has entrado a la funcion inicializacion")

    #El vector tendrá la forma
    #Xmin =#[Kp1],
           #[Kv1],
           #[Kp2],
           #[Kv2]

    #Rellenar el valor Xmin y Xmax
    Xmin = np.empty((2*objetoRobot.n,1))
    Xmin.fill(valueXMin)

    Xmax = np.empty((2*objetoRobot.n,1))
    Xmax.fill(valueXMax)

    #Arreglo para la población
    P = []

    #Arreglo para la variable randomizada
    R = np.empty([2*objetoRobot.n,1])

    #Inicializar la población aleatoriamente
    for individuo in range(objetoGenetico.NP):
        #La parte aleatoria se debe aplicar a cada variable de forma independiente, en este caso
        #Xmax-Xmin es afectado por el mismo numero aleatorio -> QUEDA PENDIENTE DEL DIA 22/07/21

        #Para cada variable del arreglo randomizar
        #Ciclo for para la randomizacion de las variables
        #Cada variable debe ser randomizada una a una pues de no hacerlo, se aplica el mismo valor de 
        #rand(0,1) a cada variable
        for variable in range(2*objetoRobot.n):
            diff = Xmax[variable]-Xmin[variable]
            #Randomizar la variable 
            r = random.uniform(0,1)*diff
            #Añadir la variable randomizada a cada renglon)
            R[variable] = r

        #R son las variables randomizadas R = rand(0,1)*(Xmax-Xmin)
        P.append(Xmin + R)
    
    return P

#2.- Mutacion
def mutacion(objGenetico,P):
    '''
        Entradas: 
            objetoGenetico -> Objeto genetico que contiene los parámetros del algoritmo genético
            P -> Población a mutar de tamaño NP
        
        Salidas:
            N -> Población de individuos que se les aplico ruido para mutarlas de tamaño NP
            

    '''

    print("Has entrado a la funcion mutacion")

    #Arreglo para los vectores ruidosos aleatorios
    N = []

    for inviduo in range(objGenetico.NP):

        #Seleccionar al azar 3 elementos del arreglo de la población P -> Acceder randomente a elemento i de P
        xa = P[random.randint(0,objGenetico.NP-1)]
        xb = P[random.randint(0,objGenetico.NP-1)]
        xc = P[random.randint(0,objGenetico.NP-1)]
        n = xc+objGenetico.F*(xa-xb)

        #Cuando el vector noisy sea negativo, iterar hasta encontrar vectores noisy que sean positivos
        while (esNegativoNoisy(n)):
            #Generar nuevamente los 3 vectores aleatorios
            xa = P[random.randint(0,objGenetico.NP-1)]
            xb = P[random.randint(0,objGenetico.NP-1)]
            xc = P[random.randint(0,objGenetico.NP-1)]
            #Actualizar el vector noisy
            n = xc+objGenetico.F*(xa-xb)

        #Llegado a este punto se garantiza que todos los vectores noisy serán positivos
        N.append(n)
    
    return N  

#3.-Recombinacion
def recombinacion(objGenetico,P,N):
    '''
        Entradas: 
            objetoGenetico -> Objeto genetico que contiene los parámetros del algoritmo genético
            P -> Población inicial de tamaño NP
            N -> Población mutada de inviduos
        
        Salidas:
            T -> Población de individuos recombinados en base al factor GR, formado por indivduos 
                 de los arreglos P y N
    '''

    print("Has entrado a la funcion recombinacion")
    #Realizar la recombinación de forma aleatoria. Dependiendo del caso se asignará al arreglo de vectores de prueba
    #Generación de arreglo de vectores de prueba
    T = []

    #Iterar para realizar la comparación
    for i in range(objGenetico.NP):
        #Si el numero generado aleatoriamente es menor al factor de recombinación el elemento N[i],
        #Será el elemento T[i]. En caso contrario: El elemento P[i], será el elemento T[i].

        #Si el numero aleatorio es menor
        if random.uniform(0,1) < objGenetico.GR:
            T.append(N[i])
        else:
            T.append(P[i])

    return T

#4.-Seleccion
def seleccion(objetoGenetico,objetoRobot,P,T,t,Q,Qd,Qdd): 
    '''
        Entrada : 
                NP -> Número de individuos de la población
                P  -> Población de la generación actual
                T  -> Vector de prueba
        Salida:
               P_i+1  -> Población de la siguiente generación
    '''
    print("Has entrado a la funcion seleccion")
    
    #PENDIENTE 14/10/21 GENERAR VARIABLES PARA ALMACENAR EL MEJOR FITNESS POR GENERACION
    # Y EL MEJOR INDIVIDUO POR GENERACION
    best_generation_fitness = 100000
    best_generation_individual = 0

    #Vector de población de la siguiente generación
    P_ip1 = []
    
    #Lista para almacenar el rendimiento de cada vector
    fitness_arreglo = []
    
    #Comparar los vectores de prueba con los originales; escoger el que tenga mayor fitness

    #Determinar el fitness de cada vector de prueba y del vector de población
    for i in range(objetoGenetico.NP):

        #Determinar el fit del vector de prueba
        fit_test = parCalculadoFIT(objetoRobot,T[i],t,Q,Qd,Qdd)

        #Determinar el fit del vector de la población
        fit_original = parCalculadoFIT(objetoRobot,P[i],t,Q,Qd,Qdd)

        #Probar que vector es mejor
        #Si el vector de prueba es menor al original, quiere decir que acumula menos error.
        #Por lo tanto, debe ser seleccionado para formar parte de la siguiente generación pues es mejor
        if fit_test < fit_original: #Gana el de prueba
            P_ip1.append(T[i]) #Añadir el vector de prueba
            fitness_arreglo.append(fit_test)
            best_fit = fit_test
            
            #print("Individuo:" +str(i+1) + ". El de prueba es mejor." + "test -> " + str(fit_test) + ". original -> " + 
               #str(fit_original))

        else:
            P_ip1.append(P[i])
            fitness_arreglo.append(fit_original)
            best_fit = fit_original
            #print("Individuo:" +str(i+1) + ". El original es mejor." + "test -> " + str(fit_test) + ". original -> " + 
              #str(fit_original))
        
        #Generar el mejor fit(CHECAR ESTO DESPUES)
        if best_fit < best_generation_fitness:
            best_generation_fitness = best_fit
            best_generation_individual = P_ip1[i]


        
    #print("El mejor individuo de la generación es: ",best_generation_individual)
        
        
        
    return P_ip1,best_generation_individual,best_generation_fitness

#FUNCION PRINCIPAL DEL ALGORITMO GENÉTICO
def algoritmoGenetico(objetoRobot,objetoGenetico,valueXMin,valueXMax,t,Q,Qd,Qdd):
    '''Entradas:
            objetoRobot(Robot) -> Robot que contiene los gdl
            objetoGenetico(Genetico) -> Objeto genetico que contiene los parámetros para el algoritmo genetico
            valueXMin(float)-> Valor minimo que podrán tomar las ganancias
            valueXMax(float) -> Valor máximo que podrán tomar las ganancias
        
        Salida:
            best_global_individual -> Vector de nx1 con las mejores ganancias
            best_global_fitness    -> El mejor fitness asociado al mejor individuo
    '''   
    print("Inicia el algoritmo Genético")
    
    #1.-Inicialización -> Solo es ejecutada una vez
    P = inicializacion(objetoRobot,objetoGenetico,valueXMin,valueXMax)
    #----EMPIEZAN LOS CICLOS DE ITERACION Inicio: 26/07/21---#
    
    #Diccionario para almacenar el mejor individuo y su fitness por generación
    #best_generation_individual_fitness = {}

    #Generar un valor global
    best_global_fitness = np.inf #Dar un valor muy grande, garantizando que siempre el primer valor será menor
    
    #Generations
    generations = 1
    
    while generations <= objetoGenetico.number_of_generations:   
        print("GENERACION ACTUAL : ",generations)
        #2.-Mutación
        N = mutacion(objetoGenetico,P)

        #3.-Recombinación
        T = recombinacion(objetoGenetico,P,N)

        #4.-Selección
        P_new,best_value,best_fitness = seleccion(objetoGenetico,objetoRobot,P,T,t,Q,Qd,Qdd)
        
        #Actualizar la poblacion actual con la poblacion nueva
        P = P_new
        
        #Almacenar el mejor fitness por g
        
        #Almacenar el mejor individuo por generacion
       # best_generation_individual_fitness[best_fitness] = best_value
        
        #Si el valor mejor de la generacion es mejor al mejor global, actualizar el mejor global
        if best_fitness < best_global_fitness:
            best_global_fitness = best_fitness
            best_global_individual = best_value
        
        #Actualizar generations
        generations +=1
            
    #print(best_generation_individual_fitness) 
    print("Finaliza el algoritmo genético")
    return best_global_individual,best_global_fitness


    
    
    #Configuración de los valores