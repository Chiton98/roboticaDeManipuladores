class Genetico:
    '''Clase para almacenar los parametros del algoritmo genetico
    
    Sus propiedades son:
        tamañoPoblacion(NP) -> Elementos de la poblacion
        numeroGeneraciones(G)  -> Número de generaciones
        tasaMutacion ->(F) Tasa de mutacion
        individuosRecombinados(nr)  -> Individuos a recombinar
        tasaRecombinacion(GR) -> Tasa de recombinacion
    
    '''
    def __init__(self,NP,G,F,nr):
        self.NP = NP
        self.number_of_generations = G
        self.F = F
        self.GR = nr/NP

        