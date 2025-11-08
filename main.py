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
def start5(): #start5 #START DAS PEDRAS
    drive_base.use_gyro(True)
    
    resetar_guinada()
    
    
    drive_base.settings(400,550) #MUDAR VELOCIDADE
    drive_base.straight(610) #ANDAR PRA FRENTE
    giroPID(30,1.5,0.001,1) #VIRAR PRO LADO
    resetar_guinada()
    drive_base.settings(200,500) #MUDAR VELOCIDADE
    drive_base.straight(125) #ANDAR PRA FRENTE EM DIREÇÃO A ATIVAÇÃO DAS PEDRAS
    drive_base.straight(-35) #ANDAR PRA TRAZ  
    giroPID(52,1.5,0.0001,1) #VIRAR PRO LADO
    resetar_guinada()
    drive_base.straight(90) #ANDAR PRA FRENTE EMPURANDO AS PEDRAS 
    motor_anexo_direita.run_angle(-300,185)
    wait(500)
    drive_base.straight(140) #ANDAR PRA FRENTE PUXANDO A ARGOLA 
    motor_anexo_direita.run_angle(300,200)
    drive_base.straight(-210) #ANDAR PRA TRAZ
    giroPID(48,1.5,0.001,1) #VIRAR PRO LADO
    resetar_guinada()
    drive_base.settings(400,1000) #MUDAR VELOCIDADE

    drive_base.straight(-160) #ANDAR PRA TRAZ EMPURANDO A ALAVANCA
    resetar_guinada()
    giroPID(-5,1.5,0.001,1) #VIRAR PRO LADO
    drive_base.straight(-20) #ANDAR PRA TRAZ EMPURANDO A ALAVANCA
    drive_base.straight(50) #ANDAR PRA FRENTE
    giroPID(40,1.5,0.001,1) #VIRAR PRO LADO EM DIREÇÃO A BASE
    resetar_guinada()
    drive_base.settings(900,5000) #MUDAR VELOCIDADE
    drive_base.straight(650) #ANDAR PRA FRENTE PARA A BASE    
    wait (50)
    drive_base.use_gyro(False)

    sm.next_state()

def start1(): #start1 #START DAS MISSÕES DA ESCAVAÇÃO SUPERFICIAL E DA REVELAÇÃO DO MAPA
    drive_base.use_gyro(True)

    resetar_guinada() 

    drive_base.settings(780,300) #DEFINIÇÃO DA VELOCIDADE PARA A RETA
    drive_base.straight(710) #ANDAR PARA FRENTE EM DIREÇÃO A MISSÃO DA ESCAVAÇÃO SUPERFICIAL

    giroPID(-38,1.5,0.001,1) #GIRAR PARA ESQUERDA
    resetar_guinada() #MUDAR GUINADA PARA ZERO
    drive_base.settings(200,200) #MUDANÇA DE VELOCIDADE PARA CONCLUIR A MISSÃO DA ESCAVAÇÃO SUPERFICIAL
    drive_base.straight(180) #IR PARA FRENTE PARA CONCLUIR A MISSÃO
    wait(300)
    motor_anexo_esquerda.run_angle(800,300) #ABAIXAR O ANEXO ESQUERDO PARA PEGAR O ARTEFATO
    drive_base.settings(800,3000)
    drive_base.straight(-200) #IR PARA TRÁS 
    resetar_guinada() #MUDAR GUINADA PARA ZERO   
    giroPID(-50,2,0.001,1) #GIRAR PARA ESQUERDA PARA IR PARA A PROXIMA MISSÃO DA REVELAÇÃO DO MAPA
    resetar_guinada() #MUDAR GUINADA PARA ZERO
    
    drive_base.settings(400,1000)
    drive_base.straight(155) #IR PARA A FRENTE EM DIREÇÃO A MISSÃO DA REVELAÇÃO DO MAPA
    motor_anexo_direita.run_angle(-800,300) #ABAIXAR O ANEXO DIREITO PARA PEGAR O ARTEFATO DA MISSÃO REVELAÇÃO DO MAPA
    drive_base.settings(800,500)
    drive_base.straight(-150) #VOLTAR PARA TRÁS
    resetar_guinada() #RESETAR GUINADA PARA ZERO
    giroPID(-60,4,0.001,1) #GIRAR PARA ESQUERDA PARA IR PARA A BASE
    drive_base.settings(800,8000) #MUDAR VELOCIDADE PRA VOLTAR A BASE
    drive_base.straight(500) #IR PARA A BASE
    wait (50)

    drive_base.use_gyro(False)

    sm.next_state()



def start6(): #start 6 #START MISSÃO GANGORRA E OQUE ESTÁ A VENDA
    drive_base.use_gyro(True)

    resetar_guinada() #MUDAR GUINADA PARA ZERO

    drive_base.settings(300,200) #DEFINIR VELOCIDADE PARA A PRIMEIRA RETA
    drive_base.straight(40) #ANDAR PARA FRENTE ANTES DO GIRO
    giroPID(-48,2,0.0003,120) #GIRAR PARA A ESQUERDA EM DIREÇÃO A MISSÃO DA GANGORRA
    wait(500)
    resetar_guinada() #DEFINIR GUINDADA PARA ZERO
    drive_base.settings(900,2000) #DEFINIR VELOCIDADE PARA A SEGUNDA RETA
    drive_base.straight(520) #ANDAR PARA FRENTE EM DIREÇÃO A MISSÃO DA GANGORRA
    wait (300)
    motor_anexo_esquerda.run_angle(600,350) #ABAIXAR O ANEXO ESQUERDO PARA ABAIXAR A ALAVANCA DA MISSÃO OQUE ESTÁ A VENDA
    motor_anexo_direita.run_angle(-400,350) #ABAIXAR O ANEXO DIREITO PARA PEGAR O ARTEFATO DA MISSÃO DA GANGORRA

    drive_base.settings(200,200) #MUDAR A VELOCIDADE  PARA A VOLTA
    drive_base.straight(-650) #VOLTAR PARA BASE

    wait (50)
    drive_base.use_gyro(False)

    sm.next_state()


def start2(): #start2 #START MISSÃO OPERAÇÃO RESGATE 
    drive_base.use_gyro(True)

    resetar_guinada()
    drive_base.settings(500,150) #DEFINIR VELOCIDADE PARA A PRIMEIRA RETA
    drive_base.straight(520) #INICIAR MOVIMENTO PARA A FRENTE EM DIREÇÃO A MISSÃO OPERAÇÃO RESGATE
    drive_base.settings(700,700) #MUDAR VELOCIDADE PARA IR PARA TRÁS

    drive_base.straight(-100) #VOLTAR PARA TRÁS
    drive_base.settings(700,300) #MUDAR VELOCIDADE PARA IR PARA FRENTE PARA CONCLUIR A MISSÃO OPERAÇÃO DE RESGATE

    drive_base.straight(270) #IR PARA FRENTE PARA CONCLUIR  A MISSÃO OPERAÇÃO DE RESGATE
    #wait (1000)
    drive_base.settings(800,5000) #MUDAR VELOCIDADE PARA IR PARA TRÁS 

    drive_base.straight(-600) #VOLTAR PARA TRÁS

    wait (50)
    drive_base.use_gyro(False)
    sm.next_state()

def start4(): #start4 #travessia
   drive_base.use_gyro(True)

   resetar_guinada()
   drive_base.settings(400,400) #MUDAR VELOCIDADE PARA IR PARA TRÁS 

   drive_base.straight(35) #INICIAR MOVIMENTO DA PRIMEIRA RETA PARA FRENTE                                      
   giroPID(30,0.7,0.0003,120) #GIRAR PARA A DIREITA EM DIREÇÃO A MISSÃO EXTRAÇÃO SEGURA
   drive_base.straight(420) #IR PARA FRENTE EM DIREÇÃO A MISSÃO EXTRAÇÃO SEGURA
   drive_base.straight(-150) #VOLTAR PARA TRÁS
   resetar_guinada() #DEFINIR GUINADA PARA ZERO
   giroPID(59,0.7,0.0003,120) #GIRAR PARA A DIREITA
   drive_base.straight(420)#IR PARA A FRENTE
   resetar_guinada()
   giroPID(-87,0.7,0.0003,120)
   motor_anexo_direita.run_time(300,1200)
   drive_base.straight(80)
   motor_anexo_direita.run_time(-300,1200)



   resetar_guinada() #DEFINIR GUINADA PARA ZERO
   giroPID(87,0.7,0.0003,120) #GIRAR PARA A DIREITA
   resetar_guinada()
   drive_base.straight(170)
   giroPID(76,0.7,0.0003,120)


   drive_base.straight(280)#IR PARA FRENTE
   drive_base.straight(-25) #VOLTAR PARA TRAS
   resetar_guinada() #DEFINIR GUINADA PARA ZERO
   giroPID(-2,2,0.003,120) #GIRAR PARA ESQUERDA
   motor_anexo_esquerda.run_angle(-800,3000) #ABAIXAR O ANEXO PARA CONCLUIR A MISSÃO EXTRAÇÃO SEGURA
   drive_base.settings(900,8000) #DEFINIR VELOCIDADE PARA A RETA

   drive_base.straight(-100) #VOLTAR PARA TRÁS
   resetar_guinada() #DEFINIR GUINADA PARA ZERO
   giroPID(-50,2,0.003,120) #GIRAR PARA ESQUERDA 
   #drive_base.settings(800,5000) #DEFINIR VELOCIDADE PARA A RETA
   drive_base.straight(700) #IR PARA FRENTE
   
   drive_base.use_gyro(False)
   sm.next_state()

def start3():  #start da compartilhada #PESCA DE ARTEFATOS
   drive_base.use_gyro(True)

   resetar_guinada() 
   drive_base.settings(300,400)
   drive_base.straight(650) #COMEÇAR A RETA
   giroPID(-90,0.7,0.0003,120) #GIRAR PARA A DIREITA
   drive_base.straight(-380) #IR PARA A FRENTE
   resetar_guinada() #DEFINIR GUINADA PARA ZERO
   giroPID(88,0.7,0.0003,120) #GIRAR PARA A ESQUERDA ANTES DO MOVIMENTO DO ANEXO
   #motor_anexo_esquerda.run_time(-500,1000) #ABAIXAR O ANEXO ESQUERDO
   #drive_base.straight(15) #ANDAR UM POUCO
   motor_anexo_esquerda.run_time(400,1200) #LEVANTAR O ANEXO PARA CONCLUIR A MISSÃO
   wait (100)
   motor_anexo_direita.run_time(-300,1000) #LEVANTA A CREMALHEIRA 

   drive_base.straight(140) #IR UM POUCO PARA FRENTE ANTES DO MOVIMENTO DOS ANEXOS PARA FINAIZAR AS MISSÕES
   motor_anexo_direita.run_time(800,2000) #ABAIXA A CREMALHEIRA 
   motor_anexo_direita.run_time(-700,2000) #LEVANTA A CREMALHEIRA
   drive_base.settings(800,8000)

   drive_base.straight(-95)
   resetar_guinada()
   giroPID(90,0.7,0.0003,120)
   drive_base.straight(-310)
   #motor_anexo_esquerda.run_time(500,1000) #ABAIXAR O ANEXO ESQUERDO


   resetar_guinada()

   giroPID(-60,1.5,0.0003,120)
   #drive_base.settings(900,3000)
   drive_base.straight(-700)
   wait (50)
   drive_base.use_gyro(False)

   sm.next_state()


''' giroPID(90,0.7,0.0003,120) #GIRAR PARA A ESQUERDA ANTES DO MOVIMENTO DO ANEXO
   resetar_guinada()
   drive_base.straight(-300)
   giroPID(-45,0.7,0.0003,120) #GIRAR PARA A ESQUERDA ANTES DO MOVIMENTO DO ANEXO'''


 

   #motor_anexo_direita.run_angle(500,300) #LEVANTAR O ANEXO PARA CONCLUIR A MISSÃO


def start7():
    drive_base.settings(450,400)
    drive_base.straight(500)
    resetar_guinada()
    giroPID(-47, 1.5, 0.0001, 1)
    drive_base.settings(500,300)
    drive_base.straight(510)
    motor_anexo_esquerda.run_angle(500, 80)
    drive_base.settings(800,8000)
    drive_base.straight(-300)
    resetar_guinada()
    giroPID(30, 1.5, 0.001, 1)
    drive_base.straight(-600)
    wait (50)
    drive_base.use_gyro(False)

    sm.next_state()



def start8():
    resetar_guinada()
    drive_base.settings(700,1000)
    drive_base.straight(810)
    giroPID(35,1.5,0.001,1)
    #motor_anexo_direita.run_time(350,1000)
    drive_base.settings(400,300)
    drive_base.straight(330)
    wait (200)
    drive_base.straight(-170)

  
def start9():
    
    drive_base.settings(460,800)
    #drive_base.straight(200)
    drive_base.straight(390)
    #motor_anexo_direita.run_time(800,800)
    wait(200)
    motor_anexo_esquerda.run_time(-800,800)
    motor_anexo_esquerda.run_time(800,800)
    wait(200)
    motor_anexo_esquerda.run_time(-800,800)
    motor_anexo_esquerda.run_time(800,800)
    wait(200)
    motor_anexo_esquerda.run_time(-800,800)
    motor_anexo_esquerda.run_time(800,800)
    wait(200)
    motor_anexo_esquerda.run_time(-800,800)
    motor_anexo_esquerda.run_time(800,800)

    drive_base.settings(900,8000)
    drive_base.straight(-400)
    sm.next_state()




#=======================EXECUÇÃO PRINCIPAL==========================#
if __name__ == "__main__":
    # Definindo os estados
    states = [
        {"name": "Start 1", "action": lambda: start1()},
        {"name": "Start 2", "action": lambda: start2()},
        {"name": "Start 3", "action": lambda: start3()},
        {"name": "Start 4", "action": lambda: start4()},
        {"name": "Start 5", "action": lambda: start5()},
        {"name": "Start 6", "action": lambda: start9()},
        {"name": "Start 7", "action": lambda: start6()},
        {"name": "Start 8", "action": lambda: start7()},
        {"name": "Start 9", "action": lambda: start8()},


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
