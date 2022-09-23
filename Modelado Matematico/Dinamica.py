import sympy as sp
#--------------------FUNCIONES PARA LA DINAMICA --------------------------

def obtenerValoresCinematicos(n):

    '''Función que calcula los arreglos de Masas,Tensores de inercia y Centros de gravedad
    en base al robot seleccionado
    
    Entrada : n -> Número de grados de libertad del robot
    
    Salidas : arregloOmegas -> Contiene las velocidades angulares desde w_0 hasta w_n
              arregloAlfas -> Contiene los aceleraciones angulares  alfa_0 hasta alfa_n
              arregloAL  -> Contiene las aceleraciones lineales de las tramas desde al_0 hasta al_n
              arreglo ALCM -> Contiene las aceleraciones lineales del CM desde alCM_0 hasta alCM_n   
              qd -> Contiene las velocidade angulares vectorialmente desde qd_1 hasta qd_N
              qdd -> Contiene las aceleraciones angulares vectorialmente desde qdd_1 hasta qd_N            
    '''
    #Arreglos a utilizar
    omega = []
    alfa = []
    al = []
    alCM = []

    qd = []
    qdd = []

    #Iterar para obtener los valores simbolicos y asignarlos a los arreglos
    for i in range(n+1):
        
        if i == 0:
            omega.append( sp.Matrix([0,0,0]))  #Velocidad angular incial
            alfa.append(sp.Matrix([0,0,0]))    #Aceleracion angular inicial
            if n == 2:
                al.append(sp.Matrix([0,sp.symbols("g"),0])) #Aceleracion lineal para el caso bidimensional
            else:
                al.append(sp.Matrix([0,0,sp.symbols("g")])) #Aceleracion lineal para el caso tridimensional

            alCM.append(0) #Aceleracion inicial del centro de masa
        
        else:
            omega.append(0)
            alfa.append(0)
            al.append(0)
            alCM.append(0)

            #Añadir las velocidades angulares
            qd.append(sp.Matrix(        [0,
                                         0,
                                        sp.symbols("\dot{q}_" + str(i))] ))

            qdd.append(sp.Matrix(       [0,
                                        0,
                                        sp.symbols("\ddot{q}_" + str(i))] ))

    return omega,alfa,al,alCM,qd,qdd                     

def obtenerValoresDinamicos(n):
    
    '''Función que calcula los arreglos de Masas,Tensores de inercia y Centros de gravedad
    en base al robot seleccionado
    
    Entrada : n -> Número de grados de libertad del robot
    
    Salidas : arregloMasas   -> Contiene las masas desde m_i hasta m_n
              arregloTenIner -> Contiene los tensores de inercia respecto al C.G. I_i hasta I_n
              arregloCentrGrav  -> Contiene los centros de gravedad de desde CG_i hasta CG_n


    '''
    
    #Arreglos
    arregloMasas = []
    arregloTenIner = []
    arregloCentrGrav = []
    
    #Iterar para asignar los valores simbolicos
    for i in range(n+1):
        if i == 0:
            arregloMasas.append(0)
            arregloTenIner.append(0)
            arregloCentrGrav.append(0)
        else:
            arregloMasas.append( sp.symbols("m" + str(i) ) )
            arregloTenIner.append( sp.Matrix([[sp.symbols( "Ixx" + str(i) ),0,0],
                                         [0,sp.symbols( "Iyy" + str(i) ),0],
                                        [0,0,sp.symbols( "Izz" + str(i) )]]) )
            #arregloTenIner.append(sp.zeros(3,3))

        #Dependiendo del robot los centros de gravedad será diferentes
       
    if n == 2: #Para el robot planar de 2 GDL
        arregloCentrGrav = [
                               sp.Matrix(  [    0,           0,0] ),
                               sp.Matrix(  [sp.symbols("Xcm1"),0,0] ),
                               sp.Matrix(  [sp.symbols("Xcm2"),0,0] ),                             
        ]
                              
      
    elif n == 3: #Para el robot antropomorfico de 3 GDL
        arregloCentrGrav = [
                               sp.Matrix(  [    0,           0,0] ),
                               sp.Matrix(  [0,0,sp.symbols("Zcm1")] ),
                               sp.Matrix(  [sp.symbols("Xcm2"),0,0] ),
                               sp.Matrix(  [sp.symbols("Xcm3"),0,0] ),                              
        ]
    elif n == 4: #PARA EL ROBOT DE 4 GDL
        arregloCentrGrav = [
                               sp.Matrix(  [    0,           0,0] ),
                               sp.Matrix(  [0,0,sp.symbols("Zcm1")] ),
                               sp.Matrix(  [sp.symbols("Xcm2"),0,0] ),
                               sp.Matrix(  [sp.symbols("Xcm3"),0,0] ),
                               sp.Matrix( [sp.symbols("Xcm4"),0,0]  )                              
        ]

    
    else:
        print("Aun no se implementa con ese robot uwu")
    

    return arregloMasas,arregloTenIner,arregloCentrGrav

def calcularIteracionesSalientes(Rs,Ps,omega,alfa,al,alCM,qd,qdd,m,I,Pcm):
    
  
    #Tamaño es igual al tamaño de MTH-1
    n = len(Rs)-1

    #Crear los arreglos que contendrán a F y a N
    F = [0]
    N = [0]

    for i in range(n):
        F.append(0)
        N.append(0)

    for i in range(0,n):
        print("Iteracion saliente ",i)
        #Velocidades angulares
        print("Calculando omega" + str(i+1))
        omega[i+1] = sp.simplify(Rs[i].T * omega[i] + qd[i])
        #Aceleraciones angulares
        alfa[i+1]  = sp.simplify(Rs[i].T * alfa[i] + (Rs[i].T*omega[i]).cross(qd[i]) + qdd[i]) 
        print("Calculando alfa" + str(i+1))
        #Aceleraciones de la i-ésima trama
        al[i+1] =    sp.simplify(Rs[i].T * ( alfa[i].cross(Ps[i]) + omega[i].cross(omega[i].cross(Ps[i])) + al[i] )) 
        print("Calculando al" + str(i+1))
        #Aceleración del centro de masa del vínculo i+1
        alCM[i+1]  = sp.simplify( alfa[i+1].cross(Pcm[i+1]) + omega[i+1].cross(omega[i+1].cross(Pcm[i+1])) + al[i+1])
        print("Calculando alCM" + str(i+1))
        #Fuerzas en el vínculo i+1
        F[i+1] = (m[i+1]*alCM[i+1])
        print("Calculando F" + str(i+1))

        #Torques en el vínculo i+1
        N[i+1] = I[i+1]*alfa[i+1] + omega[i+1].cross(I[i+1]*omega[i+1])
        print("Calculando N" + str(i+1))

    return F,N

def calcularIteracionesEntrantes(Rs,Ps,Pcm,F,N):
    
    n = len(Rs)-1
    f = []
    nt = []
    tau = []

    #Definir los arreglos de las fuerzas minusculas
    for i in range(n+1):
            f.append(0)
            nt.append(0)
            tau.append(0)

    f.append(sp.Matrix([0,0,0]))
    nt.append(sp.Matrix([0,0,0]))
  

    for i in range(n,0,-1):
        print("Iteracion entrante ",i)
        f[i] = sp.simplify(Rs[i] * f[i+1] + F[i]) #Fuerzas
        print("Calculando F" + str(i))
        nt[i] = sp.simplify(N[i] + Rs[i]*nt[i+1] + Pcm[i].cross(F[i]) + Ps[i].cross(Rs[i]*f[i+1]))
        print("Calculando nt" + str(i))
        tau[i] = nt[i][2]
        print("Calculando tau" + str(i))
    
    return tau
