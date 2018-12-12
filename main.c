
/*
 Projeto Seguidor de Linha apresentado a disciplica BCC425 Sistemas Embutidos da Universidade Federal de Ouro Preto
 Professor: Gabriel Garcia
 Alunos: Anna Cristina, Caio Andrade e Santino Bitarães
 
 */

//#####################linha de comando para upload_#######################
/*
 avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328p -c -o main.o main.c
 avr-gcc -mmcu=atmega328p main.o -o main
 avr-objcopy -O ihex -R .eeprom main main.hex
 avrdude -v -p m328p -c arduino -P /dev/cu.usbserial-1420 -b 57600 -D -U flash:w:main.hex:i
 */
//############################################


#include <avr/io.h> //carrega a biblioteca de entradas e saidas
#include <util/delay.h> //biblioteca para utilizar a funçao delay

//############ Declaracao das variaveis globais

int S1;// variavel que recebe o valor do sensor 1
int S2;// variavel que recebe o valor do sensor 2
int S3;// variavel que recebe o valor do sensor 3
int S4;// variavel que recebe o valor do sensor 4
int S5;// variavel que recebe o valor do sensor 5

int seg;// variavel de seguranca para o caso que o sensores nao leem nada ou valor aleatorio ela é 1


int P;// componente proporcional do PID
int I;// componente integradora do PID
int D;// componente derivativa do PID
int v_PID;// resultado do controlador PID, sinal de controle.

int erro;// recebe o valor do erro de acordo com o valor dos sensores
int erro_ant;// variavel para salvar o ultimo erro 


//  Variaveis de ajuste do controlador  

int vc=350;//valor definido para velocidade constante dos motores


// Constantes do controlador PID
int Kp=60; // ganho para ajuste controlador P
double Ki=0.8; // ganho que ajusta o controlador PI
int Kd=20; // ganho que ajusta a dinamica do sistema PD

int limite_motor=700;//limite para que o sinal de controle nao resulte num PWM acima do suportado pelos motores

void leitura_digital();// declaracao da funcao que le o valor dos sensores e altera as variaveis globais com valor dos sensores S..
void funcao_erro(); //calcula o erro pelos valores dos sensores e altera a variavel erro
void  calcula_PID(); // realiza o calculo do sinal de controle 

int main(){// funcao principal
    
    
    //################################## inicio da configuracao do PWM #######
    
    //configurando  modo PWM  (fase corrigida, 10 bits) do temporizador/contador TC1  que é conectado aos pinos OC1A (PB1) -> D9 e OC1B (PB2)-> D10
    
    //TC1 é um contador de 16 bits que permite grande precisão na geração de formas de onda
    
    //#################################
    //O controle do modo de operação do TC1 é feito nos registradores  TCCR1A e TCCR1B
    
    TCCR1A = ((1 << COM1A1)  | ((1 << COM1B1) | (1 << WGM11) | (1 << WGM10)));
    //TCCR1A = ((1 << COM0A1) | (1 << COM0B1) | (1 << WGM11) | (1 << WGM10)));
    //WGM11 e WGM10 configura o modo de operacao 3 do TC1 como fase corrigida, 10 bits
    //COM1A1 e COM1B1 Limpeza de OC1A/OC1B na igualdade de comparação, isso define um PWM nao invertido
    
    TCCR1B =  (1 << CS10);
    //CCS12, CCS11 e CCS10 seleção do clock para o TC1.
    
    //CCS11 e CCS10 clock/64
    
    //TCCR1B = ( (1<< CS11) | (1 << CS10));
    //CCS12, CCS11 e CCS10 seleção do clock para o TC1.
    //CCS11 e CCS10 clock/64
    
    //################################## fim da configuracao do PWM #######
    
    //################################## inicio da configuracao das portas que serao usadas #######
    DDRB=((1 <<  PB1)  |  (1 <<  PB2)); //define pino D9 e D10 com saida
    
    DDRD=((0 <<  PD2)  |  (0 <<  PD4)  |  (0 <<  PD5)  |  (0 <<  PD6)  | (0 <<  PD7));// define os pinos d2 d4 d5 d6 d7 com entrada
    //################################## fim da configuracao das portas que serao usadas #######
    
    
    int s_m9; // valor de 0 a 1023 que será escrito no motor esquerdo d9
    int s_m10;// valor de 0 a 1023 que será escrito no motor esquerdo d10
    
    
    
    erro=0;// primeiro erro e nulo para evitar qualquer outro valor q esteja nessa varivavel
    erro_ant=0;
    
    
    while(1){//LOOP INFINITO
        
        leitura_digital();//atualiza o valor lido nos sensores s1 s2 s3 s4 s5
        funcao_erro();//atualiza o valor do erro
        
        if(erro>0){// se o erro for positivo  o controle (freia) motor D10 e aumenta a rotacao do motor D9
            calcula_PID();//funcao que atualiza o valor do sinal de controle do controlador  PID
            s_m10=vc-(v_PID); // o sinal final no PWM da ponte H e um valor constante menos o sinal de controle
            s_m9=vc+(v_PID); //
        }
        
        if (erro<0){// se o erro for positivo  controle (freia) motor D9 e aumenta a rotacao do motor D10
            calcula_PID();//funcao que atualiza o valor do sinal de controle do controlador  PID
            s_m9=vc-(v_PID);// o sinal final no PWM da ponte H e um valor constante menos o sinal de controle
            s_m10=vc+(v_PID);// mantem a velocidade do motor D9
        }
        
        if (erro==0){//se o erro é nulo  anula o sinal de controle e mantem constante a velocidade dos motores
            I=0;P=0;D=0;
            s_m9=vc;
            s_m10=vc;
        }
        
        
        if (s_m9>limite_motor)  {// se o valor final a ser enviado no pwm estrapolar o limite fisico do motor mantem o valor limite 
            s_m9=limite_motor; 
        }
        
        if (s_m10>limite_motor){
            s_m10=limite_motor;
        }
        
        if(s_m9<0){// evita valores negativos na varivael q indica o sinal pwm
            s_m9=0;
        }
        if(s_m10<0){
            s_m10=0;
        }
        
        _delay_ms(10);//esse delay evita bug do sistema
        
        OCR1A = s_m9; //escreve o valor de 0 a 1023 no pino d9
        OCR1B = s_m10; //escreve o valor de 0 a 1023 no pino d10
    }
}

//####### FUNCOES %%%%%%%%%%%%%


void leitura_digital(){ //
    
    // ################ INICIO #### Leitura digital dos sensores #################
    if(PIND & (1<<PD7)) {//leitura digital do pino d7
        S1 = 1;}
    else {S1 = 0;}
    
    
    if(PIND & (1<<PD6)) {//leitura digital do pino d6
        S2 = 1;}
    else {S2 = 0;}
    
    
    if(PIND & (1<<PD5)) {//leitura digital do pino d5
        S3 = 1;}
    else {S3 = 0;}
    
    
    if(PIND & (1<<PD4)) {//leitura digital do pino d4
        S4 = 1;}
    else {S4 = 0;}
    
    
    if(PIND & (1<<PD2)) {//leitura digital do pino d2
        S5 = 1;}
    else {S5 = 0;}
    
    // ################ FIM #### Leitura digital dos sensores #################
}

void funcao_erro() {
    // ################ INICIO #### Calculo do erro #################
    
    
    if(S1 && S2 && S3 && S4 && !S5){// se o sensor s5 detectar a faixa grava o erro = -2
        erro=-2; erro_ant=erro;
        seg=0;
        
        
    }
    
    else if(S1 && S2 && S3 && !S4 && S5){// se o sensor S4 detectar a faixa grava o erro = -1
        erro=-1; erro_ant=erro;
        seg=0;
        
    }
    
    else if(S1 && S2 && !S3 && S4 && S5){// se o sensor S3 detectar a faixa grava o erro = 0
        erro=0; erro_ant=erro;
        seg=0;
        
    }
    
    else if(S1 && !S2 && S3 && S4 && S5){// se o sensor S2 detectar a faixa grava o erro = 1
        erro=1; erro_ant=erro;
        seg=0;
        
        
    }
    
    else if(!S1 && S2 && S3 && S4 && S5){// se o sensor S1 detectar a faixa grava o erro = 2
        erro=2; erro_ant=erro;
        seg=0;
        
        
    }
    else{
        
        seg=1;//varivel de seguranca, para o caso do sensor nao ler nenhuma faixa
        
    }
    
    if (seg==1){// se nao detectar a faixa mantem o erro anterior 
        
        
        erro=erro_ant;
    }
    
    
    
}


void  calcula_PID(){//funcao que calcula o PID
    int erro_ante;
    P = abs(erro);// controlador proporcional, termo proporcional ao erro
    I = I + abs(erro);// termo integrador,  relacionado com a soma de todos os valores do erro anterior
    D = abs(erro) - erro_ante;// termo derivativo, realcionado a diferença entre o erro instantâneo do ponto de ajuste, e o erro a partir do instante anterior.
    
    v_PID = (Kp * P) + (Ki * I) + (Kd * D);// efetivamente o calculo sinal de controle
    
    erro_ante= abs(erro);// salva o valor do erro anterior
    
}
