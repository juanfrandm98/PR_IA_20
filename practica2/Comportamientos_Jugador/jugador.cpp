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
Action ComportamientoJugador::think( Sensores sensores ) {

	actual.fila = sensores.posF;
	actual.columna = sensores.posC;
	actual.orientacion = sensores.sentido;

	pintarMapa( sensores );

	if( !conozcoPuntoRecarga )
		conozcoPuntoRecarga = veoPuntoInteres( sensores, recargaFila, recargaColumna, 'X' );

	if( !tengoBikini and sensores.terreno[0] == 'K' )
		tengoBikini = true;

	if( !tengoZapatillas and sensores.terreno[0] == 'D' )
		tengoZapatillas = true;

	// Calculamos el camino hasta el destino si no tenemos aun un plan
	if( !hayplan ) {
		destino.fila = sensores.destinoF;
		destino.columna = sensores.destinoC;
		hayplan = pathFinding( sensores.nivel, actual, destino, plan );
		objetivo = DESTINO;
	}

	if( sensores.nivel == 4 ) {
		if( !tengoBikini and objetivo == DESTINO ) {
			int fil, col;
			if( veoPuntoInteres( sensores, fil, col, 'K' ) ) {
				destinoPausado = destino;
				destino.fila = fil;
				destino.columna = col;
				hayplan = pathFinding( sensores.nivel, actual, destino, plan );
				objetivo = BIKINI;
			}
		}

		if( !tengoZapatillas and objetivo == DESTINO ) {
			int fil, col;
			if( veoPuntoInteres( sensores, fil, col, 'D' ) ) {
				destinoPausado = destino;
				destino.fila = fil;
				destino.columna = col;
				hayplan = pathFinding( sensores.nivel, actual, destino, plan );
				objetivo = ZAPATILLAS;
			}
		}

		if( conozcoPuntoRecarga and necesitoRecargar( sensores ) ) {
			destinoPausado = destino;
			destino.fila = recargaFila;
			destino.columna = recargaColumna;
			hayplan = pathFinding( sensores.nivel, actual, destino, plan );
			objetivo = RECARGA;
		}
	}

	Action sigAccion;
	bool accionElegida = false;
	bool replanificado = false;

	if( sensores.nivel < 4 ) {

		sigAccion = plan.front();
		plan.erase( plan.begin() );

	} else {

		do {

			// SI EL PLAN NO SE HA TERMINADO, SIGUE DE MANERA NORMAL
			if( hayplan and plan.size() > 0 ) {

				// Se escoge la siguiente acción en el plan
				sigAccion = plan.front();

				// Si se quiere ir a un lugar que requiere planificación
				if( sigAccion == actFORWARD and necesitoReplanificar( sensores ) and !replanificado ) {
					cout << "QUIERO REPLAN" << endl;
					hayplan = pathFinding( sensores.nivel, actual, destino, plan );
					replanificado = true;
				// Si se quiere ir a un lugar en el que hay un aldeano en este momento
				} else if( sigAccion == actFORWARD and EsAldeano( sensores.superficie[2] ) ) {
					sigAccion = actIDLE;
					accionElegida = true;
				// Si no hay ningún impedimento para avanzar
				} else {
					plan.erase( plan.begin() );
					accionElegida = true;
				}

			// SI EL PLAN SE HA TERMINADO Y NO SE ESTÁ RECARGANDO, SE REPLANIFICA
			} else if( sensores.posF == destino.fila and sensores.posC == destino.columna and objetivo != RECARGA ) {

				actual.fila = sensores.posF;
				actual.columna = sensores.posC;
				destino.fila = sensores.destinoF;
				destino.columna = sensores.destinoC;
				hayplan = pathFinding( sensores.nivel, actual, destino, plan );
				objetivo = DESTINO;
				sigAccion = actIDLE;
				accionElegida = true;

			// SI EL PLAN SE HA TERMINADO Y SE ESTÁ RECARGANDO, SE RECARGA HASTA QUE SE CONSIGA LA CANTIDAD DESEADA
			} else if( sensores.posF == destino.fila and sensores.posC == destino.columna and objetivo == RECARGA ) {

				// Si se necesita seguir recargando
				if( !bateriaSuficientementeLlena( sensores ) ) {
					sigAccion = actIDLE;
					accionElegida = true;
				// Si ya se ha recargado suficiente
			} else {
				destino.fila = sensores.destinoF;
				destino.columna = sensores.destinoC;
				hayplan = pathFinding( sensores.nivel, actual, destino, plan );
				objetivo = DESTINO;
				sigAccion = actIDLE;
				accionElegida = true;
			}

			// OTRA OPCIÓN INDICARÍA UN ERROR
			} else {
				cout << "ERROR EN THINK" << endl;
			}

		} while( !accionElegida );
	}

	accionesRestantes--;
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
				( a.st.fila == n.st.fila and a.st.columna == n.st.columna and a.st.orientacion == n.st.orientacion and a.bikini > n.bikini ) or
				( a.st.fila == n.st.fila and a.st.columna == n.st.columna and a.st.orientacion == n.st.orientacion and a.bikini == n.bikini and a.zapatillas > n.zapatillas ) )
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
	current.bikini = tengoBikini;
	current.zapatillas = tengoZapatillas;
	current.secuencia.empty();

	cola.push( current );

	while( !cola.empty() and ( current.st.fila != destino.fila or current.st.columna != destino.columna ) ) {

		cola.pop();

		comprobarObjetos( mapaResultado[current.st.fila][current.st.columna], current.bikini, current.zapatillas );
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
			hijoForward.coste += costeCasilla( mapaResultado[current.st.fila][current.st.columna], hijoForward.bikini, hijoForward.zapatillas );
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
		cout << "coste del plan: " << current.coste << endl;

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
				return 10;
			else
				return 100;
			break;
		case 'B':
			if( zapatillas )
				return 5;
			else
				return 50;
			break;
		case 'T':
			return 2;
			break;
		case '?':
			return 3;
			break;
		default:
			return 1;
			break;
	}

}

void ComportamientoJugador::comprobarObjetos( unsigned char casilla, bool & bikini, bool & zapatillas ) {

	if( casilla == 'K' ) {
		bikini = true;
	} else if( casilla = 'D' ) {
		zapatillas = true;
	}

}

bool ComportamientoJugador::EsAldeano(unsigned char casilla){
	if (casilla=='a')
		return true;
	else
	  return false;
}

void ComportamientoJugador::pintarMapa( Sensores sensores ) {

	switch( sensores.sentido ) {
		case norte:
			mapaResultado[sensores.posF][sensores.posC] = sensores.terreno[0];
			mapaResultado[sensores.posF - 1][sensores.posC - 1] = sensores.terreno[1];
			mapaResultado[sensores.posF - 1][sensores.posC] = sensores.terreno[2];
			mapaResultado[sensores.posF - 1][sensores.posC + 1] = sensores.terreno[3];
			mapaResultado[sensores.posF - 2][sensores.posC - 2] = sensores.terreno[4];
			mapaResultado[sensores.posF - 2][sensores.posC - 1] = sensores.terreno[5];
			mapaResultado[sensores.posF - 2][sensores.posC] = sensores.terreno[6];
			mapaResultado[sensores.posF - 2][sensores.posC + 1] = sensores.terreno[7];
			mapaResultado[sensores.posF - 2][sensores.posC + 2] = sensores.terreno[8];
			mapaResultado[sensores.posF - 3][sensores.posC - 3] = sensores.terreno[9];
			mapaResultado[sensores.posF - 3][sensores.posC - 2] = sensores.terreno[10];
			mapaResultado[sensores.posF - 3][sensores.posC - 1] = sensores.terreno[11];
			mapaResultado[sensores.posF - 3][sensores.posC] = sensores.terreno[12];
			mapaResultado[sensores.posF - 3][sensores.posC + 1] = sensores.terreno[13];
			mapaResultado[sensores.posF - 3][sensores.posC + 2] = sensores.terreno[14];
			mapaResultado[sensores.posF - 3][sensores.posC + 3] = sensores.terreno[15];
			break;
		case este:
			mapaResultado[sensores.posF][sensores.posC] = sensores.terreno[0];
			mapaResultado[sensores.posF - 1][sensores.posC + 1] = sensores.terreno[1];
			mapaResultado[sensores.posF][sensores.posC + 1] = sensores.terreno[2];
			mapaResultado[sensores.posF + 1][sensores.posC + 1] = sensores.terreno[3];
			mapaResultado[sensores.posF - 2][sensores.posC + 2] = sensores.terreno[4];
			mapaResultado[sensores.posF - 1][sensores.posC + 2] = sensores.terreno[5];
			mapaResultado[sensores.posF][sensores.posC + 2] = sensores.terreno[6];
			mapaResultado[sensores.posF + 1][sensores.posC + 2] = sensores.terreno[7];
			mapaResultado[sensores.posF + 2][sensores.posC + 2] = sensores.terreno[8];
			mapaResultado[sensores.posF - 3][sensores.posC + 3] = sensores.terreno[9];
			mapaResultado[sensores.posF - 2][sensores.posC + 3] = sensores.terreno[10];
			mapaResultado[sensores.posF - 1][sensores.posC + 3] = sensores.terreno[11];
			mapaResultado[sensores.posF][sensores.posC + 3] = sensores.terreno[12];
			mapaResultado[sensores.posF + 1][sensores.posC + 3] = sensores.terreno[13];
			mapaResultado[sensores.posF + 2][sensores.posC + 3] = sensores.terreno[14];
			mapaResultado[sensores.posF + 3][sensores.posC + 3] = sensores.terreno[15];
			break;
		case sur:
			mapaResultado[sensores.posF][sensores.posC] = sensores.terreno[0];
			mapaResultado[sensores.posF + 1][sensores.posC + 1] = sensores.terreno[1];
			mapaResultado[sensores.posF + 1][sensores.posC] = sensores.terreno[2];
			mapaResultado[sensores.posF + 1][sensores.posC - 1] = sensores.terreno[3];
			mapaResultado[sensores.posF + 2][sensores.posC + 2] = sensores.terreno[4];
			mapaResultado[sensores.posF + 2][sensores.posC + 1] = sensores.terreno[5];
			mapaResultado[sensores.posF + 2][sensores.posC] = sensores.terreno[6];
			mapaResultado[sensores.posF + 2][sensores.posC - 1] = sensores.terreno[7];
			mapaResultado[sensores.posF + 2][sensores.posC - 2] = sensores.terreno[8];
			mapaResultado[sensores.posF + 3][sensores.posC + 3] = sensores.terreno[9];
			mapaResultado[sensores.posF + 3][sensores.posC + 2] = sensores.terreno[10];
			mapaResultado[sensores.posF + 3][sensores.posC + 1] = sensores.terreno[11];
			mapaResultado[sensores.posF + 3][sensores.posC] = sensores.terreno[12];
			mapaResultado[sensores.posF + 3][sensores.posC - 1] = sensores.terreno[13];
			mapaResultado[sensores.posF + 3][sensores.posC - 2] = sensores.terreno[14];
			mapaResultado[sensores.posF + 3][sensores.posC - 3] = sensores.terreno[15];
			break;
		case oeste:
			mapaResultado[sensores.posF][sensores.posC] = sensores.terreno[0];
			mapaResultado[sensores.posF + 1][sensores.posC - 1] = sensores.terreno[1];
			mapaResultado[sensores.posF][sensores.posC - 1] = sensores.terreno[2];
			mapaResultado[sensores.posF - 1][sensores.posC - 1] = sensores.terreno[3];
			mapaResultado[sensores.posF + 2][sensores.posC - 2] = sensores.terreno[4];
			mapaResultado[sensores.posF + 1][sensores.posC - 2] = sensores.terreno[5];
			mapaResultado[sensores.posF][sensores.posC - 2] = sensores.terreno[6];
			mapaResultado[sensores.posF - 1][sensores.posC - 2] = sensores.terreno[7];
			mapaResultado[sensores.posF - 2][sensores.posC - 2] = sensores.terreno[8];
			mapaResultado[sensores.posF + 3][sensores.posC - 3] = sensores.terreno[9];
			mapaResultado[sensores.posF + 2][sensores.posC - 3] = sensores.terreno[10];
			mapaResultado[sensores.posF + 1][sensores.posC - 3] = sensores.terreno[11];
			mapaResultado[sensores.posF][sensores.posC - 3] = sensores.terreno[12];
			mapaResultado[sensores.posF - 1][sensores.posC - 3] = sensores.terreno[13];
			mapaResultado[sensores.posF - 2][sensores.posC - 3] = sensores.terreno[14];
			mapaResultado[sensores.posF - 3][sensores.posC - 3] = sensores.terreno[15];
			break;
	}

}

bool ComportamientoJugador::necesitoReplanificar( Sensores sensores ) {

	switch ( actual.orientacion ) {
		case norte:
			return mapaResultado[sensores.posF - 1][sensores.posC] == 'P' or
						 mapaResultado[sensores.posF - 1][sensores.posC] == 'M' or
						 mapaResultado[sensores.posF - 1][sensores.posC] == 'A' or
						 mapaResultado[sensores.posF - 1][sensores.posC] == 'B';
			break;
		case este:
			return mapaResultado[sensores.posF][sensores.posC + 1] == 'P' or
						 mapaResultado[sensores.posF][sensores.posC + 1] == 'M' or
						 mapaResultado[sensores.posF][sensores.posC + 1] == 'A' or
						 mapaResultado[sensores.posF][sensores.posC + 1] == 'B';
			break;
		case sur:
			return mapaResultado[sensores.posF + 1][sensores.posC] == 'P' or
						 mapaResultado[sensores.posF + 1][sensores.posC] == 'M' or
						 mapaResultado[sensores.posF + 1][sensores.posC] == 'A' or
						 mapaResultado[sensores.posF + 1][sensores.posC] == 'B';
			break;
		case oeste:
			return mapaResultado[sensores.posF][sensores.posC - 1] == 'P' or
						 mapaResultado[sensores.posF][sensores.posC - 1] == 'M' or
						 mapaResultado[sensores.posF][sensores.posC - 1] == 'A' or
						 mapaResultado[sensores.posF][sensores.posC - 1] == 'B';
			break;
	}
}

bool ComportamientoJugador::valeLaPenaRecargar( Sensores sensores ) {

	if( accionesRestantes - ( ( 1200 - sensores.bateria ) / 10 ) > 100 )
		return true;
	else
		return false;

}

int ComportamientoJugador::calcularDistancia( int f_origen, int c_origen, int f_destino, int c_destino ) {

	int f_dif, c_dif;

	if( f_origen > f_destino )
		f_dif = f_origen - f_destino;
	else
		f_dif = f_destino - f_origen;

	if( c_origen > c_destino )
		c_dif = c_origen - c_destino;
	else
		c_dif = c_destino - c_origen;

	return f_dif + c_dif;

}

bool ComportamientoJugador::destinoMuchoMasCerca( Sensores sensores ) {

	int casillasADestino = calcularDistancia( sensores.posF, sensores.posC, sensores.destinoF, sensores.destinoC );
	int casillasARecarga = calcularDistancia( sensores.posF, sensores.posC, recargaFila, recargaColumna );

	return casillasADestino + 5 < casillasARecarga;

}

bool ComportamientoJugador::necesitoRecargar( Sensores sensores ) {

	if( !valeLaPenaRecargar( sensores ) ) {
		return false;
	} else {
		// Se considera batería suficientemente cargada como para continuar con el
		// desarrollo habitual
		if( sensores.bateria > 500 ) {
			return false;
		// Se considera batería baja, antes que recargar se prioriza que el objetivo
		// esté más cerca que el punto de recarga
		} else if( sensores.bateria > 250 and sensores.bateria <= 500 ) {
			if( destinoMuchoMasCerca( sensores ) )
				return false;
			else
				return true;
		// Se considera batería crítica
		} else {
			return true;
		}
	}

}

bool ComportamientoJugador::bateriaSuficientementeLlena( Sensores sensores ) {

	return sensores.bateria >= 1200;

}

bool ComportamientoJugador::veoPuntoInteres( Sensores sensores, int & recargaFila, int & recargaColumna, unsigned char busqueda ) {

	for( int i = 0; i < 16; i++ ) {
		if( sensores.terreno[i] == busqueda ) {
			recargaFila = sensores.posF;
			recargaColumna = sensores.posC;
			calcularCoordenadas( i, recargaFila, recargaColumna );
			return true;

		}
	}

	return false;

}

void ComportamientoJugador::calcularCoordenadas( int pos, int & fila, int & columna ) {

	if( actual.orientacion == norte ) {
		switch(pos) {
			case 1:
				fila--;
				columna--;
				break;
			case 2:
				fila--;
				break;
			case 3:
				fila--;
				columna++;
				break;
			case 4:
				fila -= 2;
				columna -= 2;
				break;
			case 5:
				fila -= 2;
				columna--;
				break;
			case 6:
				fila -= 2;
				break;
			case 7:
				fila -= 2;
				columna++;
				break;
			case 8:
				fila -= 2;
				columna += 2;
				break;
			case 9:
				fila -= 3;
				columna -= 3;
				break;
			case 10:
				fila -= 3;
				columna -= 2;
				break;
			case 11:
				fila -= 3;
				columna--;
				break;
			case 12:
				fila -= 3;
				break;
			case 13:
				fila -= 3;
				columna++;
				break;
			case 14:
				fila -= 3;
				columna += 2;
				break;
			case 15:
				fila -= 3;
				columna += 3;
				break;
			default:
				break;
		}
	} else if( actual.orientacion == este ) {
		switch(pos) {
			case 1:
				fila--;
				columna++;
				break;
			case 2:
				columna++;
				break;
			case 3:
				fila++;
				columna++;
				break;
			case 4:
				fila -= 2;
				columna += 2;
				break;
			case 5:
				fila--;
				columna += 2;
				break;
			case 6:
				columna += 2;
				break;
			case 7:
				fila++;
				columna += 2;
				break;
			case 8:
				fila += 2;
				columna += 2;
				break;
			case 9:
				fila -= 3;
				columna += 3;
				break;
			case 10:
				fila -= 2;
				columna += 3;
				break;
			case 11:
				fila--;
				columna += 3;
				break;
			case 12:
				columna += 3;
				break;
			case 13:
				fila++;
				columna += 3;
				break;
			case 14:
				fila += 2;
				columna += 3;
				break;
			case 15:
				fila += 3;
				columna += 3;
				break;
			default:
				break;
		}
	} else if( actual.orientacion == sur ) {
		switch(pos) {
			case 1:
				fila++;
				columna++;
				break;
			case 2:
				fila++;
				break;
			case 3:
				fila++;
				columna--;
				break;
			case 4:
				fila += 2;
				columna += 2;
				break;
			case 5:
				fila += 2;
				columna++;
				break;
			case 6:
				fila += 2;
				break;
			case 7:
				fila += 2;
				columna--;
				break;
			case 8:
				fila += 2;
				columna -= 2;
				break;
			case 9:
				fila += 3;
				columna += 3;
				break;
			case 10:
				fila += 3;
				columna += 2;
				break;
			case 11:
				fila += 3;
				columna++;
				break;
			case 12:
				fila += 3;
				break;
			case 13:
				fila += 3;
				columna--;
				break;
			case 14:
				fila += 3;
				columna -= 2;
				break;
			case 15:
				fila += 3;
				columna -= 3;
				break;
			default:
				break;
		}
	} else {
		switch(pos) {
			case 1:
				fila++;
				columna--;
				break;
			case 2:
				columna--;
				break;
			case 3:
				fila--;
				columna--;
				break;
			case 4:
				fila += 2;
				columna -= 2;
				break;
			case 5:
				fila++;
				columna -= 2;
				break;
			case 6:
				columna -= 2;
				break;
			case 7:
				fila--;
				columna -= 2;
				break;
			case 8:
				fila -= 2;
				columna -= 2;
				break;
			case 9:
				fila += 3;
				columna -= 3;
				break;
			case 10:
				fila += 2;
				columna -= 3;
				break;
			case 11:
				fila++;
				columna -= 3;
				break;
			case 12:
				columna -= 3;
				break;
			case 13:
				fila--;
				columna -= 3;
				break;
			case 14:
				fila -= 2;
				columna -= 3;
				break;
			case 15:
				fila -= 3;
				columna -= 3;
				break;
			default:
				break;
		}
	}

}
