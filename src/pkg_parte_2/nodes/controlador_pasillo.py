# Este nodo recibe de dos topicos
# EL primer tópico es el setpoint y el segundo es el dl_dr

# Del primero se obtiene cual es el deseo  (es  siempre cero, 
# pero el controladro debe estar separado del nodo que setea el deseo)
# y del segundo obtiene la distancia a las paredes.

# Siempre que exista un setpoint vamos a revisar la distnacia y actuará el PID
# la respuesta sería en términos de velocidad angular y envía esto en el tópico

#El topico de publicacion de la velocidad angular sera /control_effort



