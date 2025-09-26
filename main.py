from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Button
from pybricks.robotics import DriveBase
from pybricks.hubs import PrimeHub
from pybricks.tools import wait


#=======================CONFIGURAÇÕES INICIAIS==========================#
hub = PrimeHub()

# 🔎 Checagem de segurança: só roda se for o HUB certo
if hub.system.info()["name"] != "V8 HUB":
    print("Hub Errado")
    while True:               # trava execução
        raise SystemExit  # encerra o programa


hub.system.set_stop_button((Button.BLUETOOTH))

left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.E)

motor_anexo_direita = Motor(Port.B)
motor_anexo_esquerda = Motor(Port.F)


# Initialize the drive base. In this example, the wheel diameter is 56mm.
# The distance between the two wheel-ground contact points is 112mm.
drive_base = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=112)

# Optionally, uncomment the line below to use the gyro for improved accuracy.
drive_base.use_gyro(True)


#=======================FUNÇÕES DE MOVIMENTO==========================#
def giroPID(angulo, kp, ki, kd):
    integral = 0
    ultimo_erro = 0
    while abs(hub.imu.heading())<abs(angulo):
        erro = angulo-hub.imu.heading()
        integral = integral + erro
        derivativa = erro-ultimo_erro
        movimento = (erro*kp)+(integral*ki)+(derivativa*kd)
        left_motor.run(movimento)
        right_motor.run(-movimento)
        ultimo_erro = erro
    left_motor.stop()
    right_motor.stop()

def resetar_guinada():
    drive_base.use_gyro(False)
    hub.imu.reset_heading(0)
    drive_base.use_gyro(True)


def config(velocidade, aceleracao):
    drive_base.settings(velocidade, aceleracao)


#=======================MÁQUINA DE ESTADO==========================#
class StateMachine:
    def __init__(self, states):
        self.states = states              # lista de estados
        self.index = 0                    # começa no primeiro estado

    def next_state(self):
        """Avança para o próximo estado"""
        self.index = (self.index + 1) % len(self.states)
        self.show_state()

    def prev_state(self):
        """Volta para o estado anterior"""
        self.index = (self.index - 1) % len(self.states)
        self.show_state()

    def run_state(self):
        """Executa a ação do estado atual"""
        state = self.states[self.index]
        print(f"Executando ação do estado: {state['name']}")    
        state["action"]()

    def show_state(self):
        """Mostra o estado atual"""

        print(f"Estado atual: {self.states[self.index]['name']}")
        print (hub.system.info()["name"])

        hub.display.number(self.index+1)



#=======================FUNÇÕES DE STARTS==========================#
def start1(): #start das pedras
    resetar_guinada()
    #drive_base.straight(200)
    
    drive_base.settings(400,500) #MUDAR VELOCIDADE
    drive_base.straight(610) #ANDAR PRA FRENTE
    giroPID(40,1.5,0.001,1) #VIRAR PRO LADO
    resetar_guinada()
    drive_base.settings(200,500) #MUDAR VELOCIDADE
    drive_base.straight(120) #ANDAR PRA FRENTE EM DIREÇÃO A ATIVAÇÃO DAS PEDRAS
    drive_base.straight(-20) #ANDAR PRA TRAZ  
    giroPID(48,1.5,0.0001,1) #VIRAR PRO LADO
    resetar_guinada()
    drive_base.straight(90) #ANDAR PRA FRENTE EMPURANDO AS PEDRAS 
    motor_anexo_direita.run_angle(-300,185)
    wait(500)
    drive_base.straight(140) #ANDAR PRA FRENTE PUXANDO A ARGOLA 
    motor_anexo_direita.run_angle(300,200)
    drive_base.straight(-230) #ANDAR PRA TRAZ
    giroPID(48,1.5,0.001,1) #VIRAR PRO LADO
    resetar_guinada()
    drive_base.settings(200,300) #MUDAR VELOCIDADE

    drive_base.straight(-140) #ANDAR PRA TRAZ EMPURANDO A ALAVANCA
    drive_base.straight(50) #ANDAR PRA FRENTE
    giroPID(34,1.5,0.001,1) #VIRAR PRO LADO EM DIREÇÃO A BASE
    resetar_guinada()
    drive_base.settings(800,800) #MUDAR VELOCIDADE
    drive_base.straight(650) #ANDAR PRA FRENTE PARA A BASE 
    

    
 


    
    
    sm.next_state()

def start2(): #START DO TRIDENT
    resetar_guinada()

    drive_base.settings(500,300) #MUDAR VELOCIDADE
    drive_base.straight(720)#ANDAR PARA FRENTE
    #drive_base.straight(-150)#ANDAR PARA TRAZ
    #drive_base.straight(250)#ANDAR PARA FRENTE
    giroPID(-38,1,0.001,1)#VIRAR PRO LADO
    resetar_guinada()
    drive_base.settings(200,200)#MUDAR VELOCIDADE
    drive_base.straight(180)#ANDAR PARA FRENTE
    motor_anexo_esquerda.run_angle(300,300)
    drive_base.straight(-200)
    resetar_guinada()
    giroPID(-57,4,0.001,1)
    resetar_guinada()

    drive_base.straight(155)
    motor_anexo_direita.run_angle(-300,300)
    drive_base.straight(-150)
    resetar_guinada()
    giroPID(-65,2,0.001,1)
    drive_base.settings(800,8000)
    drive_base.straight(500)
    sm.next_state()



def start3(): #start gangorra
    resetar_guinada()

    drive_base.settings(300,200) #MUDAR VELOCIDADE
    drive_base.straight(40) #ANDAR PRA FRENTE
    giroPID(-41,0.7,0.0003,120) #VIRAR PRO LADO
    wait(500)
    print (hub.imu.heading())
    resetar_guinada()
    drive_base.settings(900,2000) #MUDAR VELOCIDADE
    drive_base.straight(520) #ANDAR PRA FRENTE
    wait (300)
    motor_anexo_esquerda.run_angle(600,350)
    motor_anexo_direita.run_angle(-400,350)
    #wait (300)
    #motor_anexo_esquerda.run_angle(-600,200)

    drive_base.settings(750,80)
    drive_base.straight(-600)
    sm.next_state()


def start4(): #start areia
    resetar_guinada()

    drive_base.settings(500,150) #MUDAR VELOCIDADE
    drive_base.straight(520) #ANDAR PRA FRENTE
    drive_base.settings(700,700) #MUDAR VELOCIDADE

    drive_base.straight(-100) #ANDAR PRA FRENTE
    drive_base.settings(700,300) #MUDAR VELOCIDADE

    drive_base.straight(270) #ANDAR PRA FRENTE
    wait (1000)
    drive_base.settings(800,300) #MUDAR VELOCIDADE

    drive_base.straight(-600) #ANDAR PRA FRENTE



    sm.next_state
#=======================EXECUÇÃO PRINCIPAL==========================#
if __name__ == "__main__":
    # Definindo os estados
    states = [
        {"name": "Start 1", "action": lambda: start1()},
        {"name": "Start 2", "action": lambda: start2()},
        {"name": "Start 3", "action": lambda: start3()},
        {"name": "Start 4", "action": lambda: start4()},
    ]

    sm = StateMachine(states)
    sm.show_state()

    # Loop infinito de estados.
    while True:
        if Button.RIGHT in hub.buttons.pressed():   # botão direito
            sm.next_state()
            wait (200)
        elif Button.LEFT in hub.buttons.pressed(): # botão esquerdo
            sm.prev_state()
            wait (200)
        elif Button.CENTER in hub.buttons.pressed(): # botão do meio
            sm.run_state()
            wait (200)
        elif Button.CENTER in hub.buttons.pressed() and Button.BLUETOOTH in hub.buttons.pressed(): # sair
            print("Encerrando...")
            break
