#ifndef COMPORTAMIENTOJUGADOR_H
#define COMPORTAMIENTOJUGADOR_H

#include "comportamientos/comportamiento.hpp"

#include <list>

struct estado {
  int fila;
  int columna;
  int orientacion;
};

enum ObjetivoFijado {
  DESTINO,
  RECARGA,
  ZAPATILLAS,
  BIKINI
};

class ComportamientoJugador : public Comportamiento {
  public:
    ComportamientoJugador(unsigned int size) : Comportamiento(size) {
      // Inicializar Variables de Estado
      fil = col = 99;
      brujula = 0; // 0: Norte, 1:Este, 2:Sur, 3:Oeste
      destino.fila = -1;
      destino.columna = -1;
      destino.orientacion = -1;
      hayplan=false;
      tengoBikini = false;
      tengoZapatillas = false;
      conozcoPuntoRecarga = false;
      recargaFila = recargaColumna = -1;
      accionesRestantes = 3000;
    }
    ComportamientoJugador(std::vector< std::vector< unsigned char> > mapaR) : Comportamiento(mapaR) {
      // Inicializar Variables de Estado
      fil = col = 99;
      brujula = 0; // 0: Norte, 1:Este, 2:Sur, 3:Oeste
      destino.fila = -1;
      destino.columna = -1;
      destino.orientacion = -1;
      hayplan=false;
      tengoBikini = false;
      tengoZapatillas = false;
      conozcoPuntoRecarga = false;
      recargaFila = recargaColumna = -1;
      accionesRestantes = 3000;
    }
    ComportamientoJugador(const ComportamientoJugador & comport) : Comportamiento(comport){}
    ~ComportamientoJugador(){}

    Action think(Sensores sensores);
    int interact(Action accion, int valor);
    void VisualizaPlan(const estado &st, const list<Action> &plan);
    ComportamientoJugador * clone(){return new ComportamientoJugador(*this);}

  private:
    // Declarar Variables de Estado
    int fil, col, brujula;
    estado actual, destino, destinoPausado;
    list<Action> plan;
    bool hayplan;
    bool tengoBikini;
    bool tengoZapatillas;
    ObjetivoFijado objetivo;
    bool conozcoPuntoRecarga;
    int recargaFila;
    int recargaColumna;
    int accionesRestantes;

    // Métodos privados de la clase
    bool pathFinding(int level, const estado &origen, const estado &destino, list<Action> &plan);
    bool pathFinding_Profundidad(const estado &origen, const estado &destino, list<Action> &plan);
    bool pathFinding_Anchura( const estado & origen, const estado & destino, list<Action> & plan );
    bool pathFinding_Costo_Uniforme( const estado & origen, const estado & destino, list<Action> &plan );

    void PintaPlan(list<Action> plan);
    bool HayObstaculoDelante(estado &st);
    void calcularCoordenadasAvance( const estado st, int & fil, int & col );
    int costeCasilla( unsigned char casilla, bool bikini, bool zapatillas );
    void comprobarObjetos( unsigned char casilla, bool & bikini, bool & zapatillas );
    bool EsAldeano( unsigned char casilla );
    void pintarMapa( Sensores sensores );
    bool necesitoReplanificar( Sensores sensores );
    bool necesitoRecargar( Sensores sensores );
    bool bateriaSuficientementeLlena( Sensores sensores );
    bool veoPuntoInteres( Sensores sensores, int & recargaFila, int & recargaColumna, unsigned char busqueda );
    void calcularCoordenadas( int pos, int & fila, int & columna );
    bool valeLaPenaRecargar( Sensores sensores );
    bool destinoMuchoMasCerca( Sensores sensores );
    int calcularDistancia( int f_origen, int c_origen, int f_destino, int c_destino );

};

#endif
