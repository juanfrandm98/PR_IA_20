#include "../Comportamientos_Jugador/jugador.hpp"
#include "motorlib/util.h"

#include <iostream>
#include <cmath>
#include <set>
#include <stack>
#include <queue>


// Este es el método principal que debe contener los 4 Comportamientos_Jugador
// que se piden en la práctica. Tiene como entrada la información de los
// sensores y devuelve la acción a realizar.
/*
Action ComportamientoJugador::think(Sensores sensores) {
	Action accion = actIDLE;
	// Estoy en el nivel 1

	actual.fila        = sensores.posF;
	actual.columna     = sensores.posC;
	actual.orientacion = sensores.sentido;

	cout << "Fila: " << actual.fila << endl;
	cout << "Col : " << actual.columna << endl;
	cout << "Ori : " << actual.orientacion << endl;

	destino.fila       = sensores.destinoF;
	destino.columna    = sensores.destinoC;

	if (sensores.nivel != 4){
		bool hay_plan = pathFinding (sensores.nivel, actual, destino, plan);
	}
	else {
		// Estoy en el nivel 2
		cout << "Aún no implementado el nivel 2" << endl;
	}

  return accion;
}*/
Action ComportamientoJugador::think( Sensores sensores ) {

	// Calculamos el camino hasta el destino si no tenemos aun un plan
	if( !hayplan ) {
		actual.fila = sensores.posF;
		actual.columna = sensores.posC;
		actual.orientacion = sensores.sentido;
		destino.fila = sensores.destinoF;
		destino.columna = sensores.destinoC;
		hayplan = pathFinding( sensores.nivel, actual, destino, plan );
	}

	Action sigAccion = actIDLE;

	if( hayplan and plan.size() > 0 ) {		// Hay un plan no vacío
		sigAccion = plan.front();						// Tomamos la siguiente acción del hayPlan
		plan.erase( plan.begin() );					// Eliminamos la accion de la lista de acciones
	} else {
		// Aquí solo entra cuando no es posible encontrar un comportamiento o está
		// mal implementado el método de búsqueda
	}

	return sigAccion;

}


// Llama al algoritmo de busqueda que se usará en cada comportamiento del agente
// Level representa el comportamiento en el que fue iniciado el agente.
bool ComportamientoJugador::pathFinding (int level, const estado &origen, const estado &destino, list<Action> &plan){
	switch (level){
		case 1: cout << "Busqueda en profundad\n";
			      return pathFinding_Profundidad(origen,destino,plan);
						break;
		case 2: cout << "Busqueda en Anchura\n";
			      return pathFinding_Anchura( origen, destino, plan );
						break;
		case 3: cout << "Busqueda Costo Uniforme\n";
						return pathFinding_Costo_Uniforme( origen, destino, plan );
						break;
		case 4: cout << "Busqueda para el reto\n";
						return pathFinding_Costo_Uniforme( origen, destino, plan );
						break;
	}
	cout << "Comportamiento sin implementar\n";
	return false;
}


//---------------------- Implementación de la busqueda en profundidad ---------------------------

// Dado el código en carácter de una casilla del mapa dice si se puede
// pasar por ella sin riegos de morir o chocar.
bool EsObstaculo(unsigned char casilla){
	if (casilla=='P' or casilla=='M')
		return true;
	else
	  return false;
}


// Comprueba si la casilla que hay delante es un obstaculo. Si es un
// obstaculo devuelve true. Si no es un obstaculo, devuelve false y
// modifica st con la posición de la casilla del avance.
bool ComportamientoJugador::HayObstaculoDelante(estado &st){
	int fil=st.fila, col=st.columna;

  // calculo cual es la casilla de delante del agente
	switch (st.orientacion) {
		case 0: fil--; break;
		case 1: col++; break;
		case 2: fil++; break;
		case 3: col--; break;
	}

	// Compruebo que no me salgo fuera del rango del mapa
	if (fil<0 or fil>=mapaResultado.size()) return true;
	if (col<0 or col>=mapaResultado[0].size()) return true;

	// Miro si en esa casilla hay un obstaculo infranqueable
	if (!EsObstaculo(mapaResultado[fil][col])){
		// No hay obstaculo, actualizo el parámetro st poniendo la casilla de delante.
    st.fila = fil;
		st.columna = col;
		return false;
	}
	else{
	  return true;
	}
}




struct nodo{
	estado st;
	int coste;
	bool bikini;
	bool zapatillas;
	list<Action> secuencia;

	bool operator< ( const nodo &n ) const {
		return coste > n.coste;
	}
};

struct ComparaEstados{
	bool operator()(const estado &a, const estado &n) const{
		if ((a.fila > n.fila) or (a.fila == n.fila and a.columna > n.columna) or
	      (a.fila == n.fila and a.columna == n.columna and a.orientacion > n.orientacion))
			return true;
		else
			return false;
	}
};

struct ComparaNodos {
	bool operator() ( const nodo &a, const nodo &n ) const {
		if( ( a.st.fila > n.st.fila ) or
				( a.st.fila == n.st.fila and a.st.columna > n.st.columna ) or
				( a.st.fila == n.st.fila and a.st.columna == n.st.columna and a.st.orientacion > n.st.orientacion ) or
				( a.st.fila == n.st.fila and a.st.columna == n.st.columna and a.st.orientacion == n.st.orientacion and a.bikini != n.bikini ) or
				( a.st.fila == n.st.fila and a.st.columna == n.st.columna and a.st.orientacion == n.st.orientacion and a.bikini == n.bikini and a.zapatillas != n.zapatillas ) or
			 	( a.st.fila == n.st.fila and a.st.columna == n.st.columna and a.st.orientacion == n.st.orientacion and a.bikini == n.bikini and a.zapatillas == n.zapatillas and a.coste > n.coste ) )
				return true;
		else
				return false;
	}
};

// Implementación de la búsqueda en profundidad.
// Entran los puntos origen y destino y devuelve la
// secuencia de acciones en plan, una lista de acciones.
bool ComportamientoJugador::pathFinding_Profundidad(const estado &origen, const estado &destino, list<Action> &plan) {
	//Borro la lista
	cout << "Calculando plan\n";
	plan.clear();
	set<estado,ComparaEstados> generados; // Lista de Cerrados
	stack<nodo> pila;											// Lista de Abiertos

  nodo current;
	current.st = origen;
	current.secuencia.empty();

	pila.push(current);

  while (!pila.empty() and (current.st.fila!=destino.fila or current.st.columna != destino.columna)){

		pila.pop();
		generados.insert(current.st);

		// Generar descendiente de girar a la derecha
		nodo hijoTurnR = current;
		hijoTurnR.st.orientacion = (hijoTurnR.st.orientacion+1)%4;
		if (generados.find(hijoTurnR.st) == generados.end()) {
			hijoTurnR.secuencia.push_back(actTURN_R);
			pila.push(hijoTurnR);
		}

		// Generar descendiente de girar a la izquierda
		nodo hijoTurnL = current;
		hijoTurnL.st.orientacion = (hijoTurnL.st.orientacion+3)%4;
		if (generados.find(hijoTurnL.st) == generados.end()){
			hijoTurnL.secuencia.push_back(actTURN_L);
			pila.push(hijoTurnL);
		}

		// Generar descendiente de avanzar
		nodo hijoForward = current;
		if (!HayObstaculoDelante(hijoForward.st)){
			if (generados.find(hijoForward.st) == generados.end()){
				hijoForward.secuencia.push_back(actFORWARD);
				pila.push(hijoForward);
			}
		}

		// Tomo el siguiente valor de la pila
		if (!pila.empty()){
			current = pila.top();
		}
	}

  cout << "Terminada la busqueda\n";

	if (current.st.fila == destino.fila and current.st.columna == destino.columna){
		cout << "Cargando el plan\n";
		plan = current.secuencia;
		cout << "Longitud del plan: " << plan.size() << endl;
		PintaPlan(plan);
		// ver el plan en el mapa
		VisualizaPlan(origen, plan);
		return true;
	}
	else {
		cout << "No encontrado plan\n";
	}


	return false;
}

// Implementación de la búsqueda en anchura.
// Entran los puntos origen y destino y devuelve la secuencia de acciones en
// plan, una lista de acciones.
bool ComportamientoJugador::pathFinding_Anchura( const estado & origen, const estado & destino, list<Action> &plan ) {

	// Borro la lista
	cout << "Calculando plan\n";
	plan.clear();
	set<estado,ComparaEstados> generados; // Lista de Cerrados
	queue<nodo> cola;											// Lista de abiertos

	nodo current;
	current.st = origen;
	current.secuencia.empty();

	cola.push( current );

	while( !cola.empty() and ( current.st.fila != destino.fila or current.st.columna != destino.columna ) ) {

		cola.pop();
		generados.insert( current.st );

		// Generar descendiente de girar a la derecha
		nodo hijoTurnR = current;
		hijoTurnR.st.orientacion = ( hijoTurnR.st.orientacion + 1 ) % 4;

		if( generados.find( hijoTurnR.st ) == generados.end() ) {
			hijoTurnR.secuencia.push_back( actTURN_R );
			cola.push( hijoTurnR );
		}

		// Generar descendiente a la izquierda
		nodo hijoTurnL = current;
		hijoTurnL.st.orientacion = ( hijoTurnL.st.orientacion + 3 ) % 4;

		if( generados.find( hijoTurnL.st ) == generados.end() ) {
			hijoTurnL.secuencia.push_back( actTURN_L );
			cola.push( hijoTurnL );
		}

		// Generar descendiente de avanzar
		nodo hijoForward = current;
		if( !HayObstaculoDelante( hijoForward.st ) )
			if( generados.find( hijoForward.st ) == generados.end() ) {
				hijoForward.secuencia.push_back( actFORWARD );
				cola.push( hijoForward );
			}

		// Tomo el siguiente valor de la cola
		if( !cola.empty() ) {
			current = cola.front();
			while( generados.find(current.st) != generados.end() ) {
				cola.pop();
				current = cola.front();
			}
		}

	}

	cout << "Terminada la búsqueda\n";

	if( current.st.fila == destino.fila and current.st.columna == destino.columna ) {
		cout << "Cargando el plan\n";
		plan = current.secuencia;
		cout << "longitud del plan: " << plan.size() << endl;

		PintaPlan( plan );
		VisualizaPlan( origen, plan );

		return true;
	} else {
		cout << "No encontrado plan\n";
		return false;
	}

}

// Implementación de la búsqueda en costo uniforme.
// Entran los puntos origen y destino y devuelve la secuencia de acciones en
// plan, una lista de acciones.
bool ComportamientoJugador::pathFinding_Costo_Uniforme( const estado & origen, const estado & destino, list<Action> &plan ) {

	// Borro la lista
	cout << "Calculando plan\n";
	plan.clear();
	set<nodo,ComparaNodos> generados; // Lista de Cerrados
	priority_queue<nodo> cola;						// Lista de abiertos

	nodo current;
	current.st = origen;
	current.coste = 0;
	comprobarObjetos( mapaResultado[current.st.fila][current.st.columna], current.bikini, current.zapatillas );
	current.secuencia.empty();

	cola.push( current );

	while( !cola.empty() and ( current.st.fila != destino.fila or current.st.columna != destino.columna ) ) {

		cola.pop();
		generados.insert( current );

		// Generar descendiente de girar a la derecha
		nodo hijoTurnR = current;
		hijoTurnR.st.orientacion = ( hijoTurnR.st.orientacion + 1 ) % 4;
		hijoTurnR.coste += costeCasilla( mapaResultado[hijoTurnR.st.fila][hijoTurnR.st.columna], hijoTurnR.bikini, hijoTurnR.zapatillas );

		if( generados.find( hijoTurnR ) == generados.end() ) {
			hijoTurnR.secuencia.push_back( actTURN_R );
			cola.push( hijoTurnR );
		}

		// Generar descendiente a la izquierda
		nodo hijoTurnL = current;
		hijoTurnL.st.orientacion = ( hijoTurnL.st.orientacion + 3 ) % 4;
		hijoTurnL.coste += costeCasilla( mapaResultado[hijoTurnL.st.fila][hijoTurnL.st.columna], hijoTurnL.bikini, hijoTurnL.zapatillas );

		if( generados.find( hijoTurnL ) == generados.end() ) {
			hijoTurnL.secuencia.push_back( actTURN_L );
			cola.push( hijoTurnL );
		}

		// Generar descendiente de avanzar
		nodo hijoForward = current;
		if( !HayObstaculoDelante( hijoForward.st ) ) {
			int fil, col;
			calcularCoordenadasAvance( hijoForward.st, fil, col );
			hijoForward.coste += costeCasilla( mapaResultado[fil][col], hijoForward.bikini, hijoForward.zapatillas );
			if( !hijoForward.bikini or !hijoForward.zapatillas )
				comprobarObjetos( mapaResultado[fil][col], hijoForward.bikini, hijoForward.zapatillas );

			if( generados.find( hijoForward ) == generados.end() ) {
				hijoForward.secuencia.push_back( actFORWARD );
				cola.push( hijoForward );
			}
		}

		// Tomo el siguiente valor de la cola
		if( !cola.empty() ) {
			current = cola.top();
			while( generados.find(current) != generados.end() ) {
				cola.pop();
				current = cola.top();
			}
		}

	}

	cout << "Terminada la búsqueda\n";

	if( current.st.fila == destino.fila and current.st.columna == destino.columna ) {
		cout << "Cargando el plan\n";
		plan = current.secuencia;
		cout << "longitud del plan: " << plan.size() << endl;

		PintaPlan( plan );
		VisualizaPlan( origen, plan );

		return true;
	} else {
		cout << "No encontrado plan\n";
		return false;
	}

}

// Sacar por la términal la secuencia del plan obtenido
void ComportamientoJugador::PintaPlan(list<Action> plan) {
	auto it = plan.begin();
	while (it!=plan.end()){
		if (*it == actFORWARD){
			cout << "A ";
		}
		else if (*it == actTURN_R){
			cout << "D ";
		}
		else if (*it == actTURN_L){
			cout << "I ";
		}
		else {
			cout << "- ";
		}
		it++;
	}
	cout << endl;
}



void AnularMatriz(vector<vector<unsigned char> > &m){
	for (int i=0; i<m[0].size(); i++){
		for (int j=0; j<m.size(); j++){
			m[i][j]=0;
		}
	}
}


// Pinta sobre el mapa del juego el plan obtenido
void ComportamientoJugador::VisualizaPlan(const estado &st, const list<Action> &plan){
  AnularMatriz(mapaConPlan);
	estado cst = st;

	auto it = plan.begin();
	while (it!=plan.end()){
		if (*it == actFORWARD){
			switch (cst.orientacion) {
				case 0: cst.fila--; break;
				case 1: cst.columna++; break;
				case 2: cst.fila++; break;
				case 3: cst.columna--; break;
			}
			mapaConPlan[cst.fila][cst.columna]=1;
		}
		else if (*it == actTURN_R){
			cst.orientacion = (cst.orientacion+1)%4;
		}
		else {
			cst.orientacion = (cst.orientacion+3)%4;
		}
		it++;
	}
}



int ComportamientoJugador::interact(Action accion, int valor){
  return false;
}

void ComportamientoJugador::calcularCoordenadasAvance( const estado st, int & fil, int & col ) {

	fil=st.fila;
	col=st.columna;

	switch (st.orientacion) {
		case 0: fil--; break;
		case 1: col++; break;
		case 2: fil++; break;
		case 3: col--; break;
	}

}

int ComportamientoJugador::costeCasilla( unsigned char casilla, bool bikini, bool zapatillas ) {

	switch ( casilla ) {
		case 'A':
			if( bikini )
				return 100;
			else
				return 10;
			break;
		case 'B':
			if( zapatillas )
				return 50;
			else
				return 5;
			break;
		case 'T':
			return 2;
			break;
		default:
			return 1;
			break;
	}

}

void ComportamientoJugador::comprobarObjetos( unsigned char casilla, bool & bikini, bool & zapatillas ) {

	if( casilla == 'K' )
		bikini = true;
	else if( casilla = 'D' )
		zapatillas = true;

}
