import sympy as sp

def obtenerParametrosRobot(GDL):
    if GDL == 1: #Robot Planar (R)  
        q1,a1 = sp.symbols("q1,a1")
        Alfas = [0]
        As    = [a1]
        Ds = [0]
        Qs = [q1]
    
    elif GDL == 2: #Robot Planar (RR)
        q1,q2,L1,L2 = sp.symbols("q1,q2,L1,L2")
        Alfas = [0,0]
        As    = [0,L1]
        Ds = [0,0]
        Qs = [q1,q2]   

    elif GDL == 3: #Robot Antropomorfico (RRR)
        q1,q2,q3,L1,L2,L3 = sp.symbols("q1,q2,q3,L1,L2,L3")
        Alfas = [0,sp.pi/2,0]
        As    = [0,0,L2]
        Ds = [L1,0,0]
        Qs = [q1,q2,q3]  
    
    elif GDL == 4: #ROBOT 4 GRADOS DE LIBERTAD(RRRR)
        q1,q2,q3,q4,L1,L2,L3 = sp.symbols("q1,q2,q3,q4,L1,L2,L3")
        Alfas = [0,sp.pi/2,0,0]
        As = [0,0,L2,L3]
        Ds = [L1,0,0,0]
        Qs = [q1,q2,q3,q4]

    elif GDL == 6: #Robot Antropomorfico con muñeca de EULER
        q1,q2,q3,q4,q5,q6,L1,L2,L3,L4 = sp.symbols("q1,q2,q3,q4,q5,q6,L1,L2,L3,L4")
        Alfas = [sp.pi/2,0,sp.pi/2,-sp.pi/2,sp.pi/2,0]
        As    = [0,L2,0,0,0,0]
        Ds = [L1,0,0,L3,0,0]
        Qs = [q1,q2,q3+sp.pi/2,q4,q5,q6]

    else:
        print("Funcionalidad aun no es implementada")

    return As,Alfas,Ds,Qs

def obtenerMTHArticulaciones(alfa,a,d,q):

    r1 = [sp.cos(q),-sp.sin(q),0,a]
    r2 = [sp.sin(q)*(sp.cos(alfa)),sp.cos(q)*(sp.cos(alfa)),(-sp.sin(alfa)),(-sp.sin(alfa))*d]
    r3 = [sp.sin(q)*(sp.sin(alfa)),sp.cos(q)*(sp.sin(alfa)) ,(sp.cos(alfa)) ,(sp.cos(alfa))*d]
    r4 = [0,0,0,1]

    return sp.Matrix([r1,r2,r3,r4])

def multMatrizRecursivamente(arregloMTH,i):

    if i == 1:
        return arregloMTH[i-1]
    
    else:
        return multMatrizRecursivamente(arregloMTH,i-1)*arregloMTH[i-1]       