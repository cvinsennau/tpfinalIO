#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 24 15:14:51 2020

@author: cande
"""

from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
import datetime
import locale
from locale import atof
import os
import math
from math import factorial



#Coordenadas
Cantidad_pedidos=100

x_entregas=np.zeros(Cantidad_pedidos)
y_entregas=np.zeros(Cantidad_pedidos)

#Defino una funcion que me tire un punto en el eje x
def punto_x():
    return np.random.uniform(0,605)

#Same para el eje y
def punto_y():
    return np.random.uniform(0,150)

#defino semilla para tener repetibilidad en los puntos generados
np.random.seed(0)

#relleno el array con puntos aleatorios x,y
for i in range(Cantidad_pedidos):
    x_entregas[i]=punto_x()
    y_entregas[i]=punto_y()

#ahora creamos los locales, de igual manera que los pedidos
cantidad_locales = 4
x_vendedor= np.zeros(cantidad_locales)
y_vendedor= np.zeros(cantidad_locales)

np.random.seed(0)
for i in range(cantidad_locales):
    x_vendedor[i] = punto_x()
    y_vendedor[i] = punto_y()

#Ahora, junto los puntos de los vendedores con los de entregas en un mismo array
#al cual llamo x_coordenadas e y_coordenadas
x_coordenadas = np.append(x_vendedor,x_entregas)
y_coordenadas = np.append(y_vendedor,y_entregas)


#Guardar arrays para tener los mismos puntos
#np.save('x_entregas.npy',x_entregas)
#np.save('x_vendedor.npy',x_vendedor)
#np.save('y_entregas.npy',y_entregas)
#np.save('y_vendedor.npy',y_vendedor)


##Grafico
plt.scatter(x_entregas,y_entregas,marker='.')
plt.scatter(x_vendedor,y_vendedor,marker='o')
plt.axis('scaled')  #para que tengan la misma escala
plt.grid()
plt.xlim(0,650)
plt.ylim(0,180)
plt.ylabel('coordenada y')
plt.xlabel('coordenada x')
plt.title('Ubicación de los pedidos y locales')
plt.show()


#Matriz de distancia
N = len(x_coordenadas)          #puede ser y_coordenadas tmb, tienen el mismo tamaño

#Creo una matriz de tamaño N*N con valores 0
distancia = np.zeros((N,N))

#Ahora vamos a tomar la distancia de cada punto i, a cada punto j
for i in range(N):
    for j in range(N):
        distancia[i,j] = math.dist([x_coordenadas[i],y_coordenadas[i]], [x_coordenadas[j],y_coordenadas[j]])

print(distancia)



#Pickup deliveries

#creo una matriz de tamaño Cantidad_pedidos,2:
pickup_deliveries=np.zeros((Cantidad_pedidos,2))

#Para que se elija aleatoriamente el local, usamos np.random.choice()
#Entonces, necesito un array con los valores [0,1,2,3]
num_local=np.zeros(len(x_vendedor))
for a in range(len(x_vendedor)):
    num_local[a]=a
#Obs: uso x_vendedor como podria usar y_vendedor. tienen el mismo tamaño

#Ahora voy con la matriz pickup_deliveries

#Para cada pedido en el rango [4,103]:
    #asigname un local, cualquiera de los 4
    #registrame en la otra columna el pedido que es

#Entonces, defino num_pedido para que vaya de 4 a 103    
for i in range (len(pickup_deliveries)):
    num_pedido=i+len(x_vendedor)
    pickup_deliveries[i,0]=np.random.choice(num_local)
    pickup_deliveries[i,1]=num_pedido

#Guardo el array pickup_deliveries
#np.save('pickup_deliveries.npy',pickup_deliveries)

#%%

##OR TOOLS - PICKUP & DELIVERIES


"""Simple Pickup Delivery Problem (PDP)."""

from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data['distance_matrix'] = distancia
    data['pickups_deliveries'] = pickup_deliveries
    data['num_vehicles'] = 4
    data['depot'] = 0
    return data


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    total_distance = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += ' {} -> '.format(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        plan_output += '{}\n'.format(manager.IndexToNode(index))
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        print(plan_output)
        total_distance += route_distance
    print('Total Distance of all routes: {}m'.format(total_distance))


def main():
    """Entry point of the program."""
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)


    # Define cost of each arc.
    def distance_callback(from_index, to_index):
        """Returns the manhattan distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance constraint.
    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        3000,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Define Transportation Requests.
    for request in data['pickups_deliveries']:
        pickup_index = manager.NodeToIndex(request[0])
        delivery_index = manager.NodeToIndex(request[1])
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        routing.solver().Add(
            routing.VehicleVar(pickup_index) == routing.VehicleVar(
                delivery_index))
        routing.solver().Add(
            distance_dimension.CumulVar(pickup_index) <=
            distance_dimension.CumulVar(delivery_index))

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)


if __name__ == '__main__':
    main()












