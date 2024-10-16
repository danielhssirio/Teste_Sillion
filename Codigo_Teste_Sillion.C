#include <avr/io.h>
#include <util/delay.h>

#define MOTOR_PIN PB0  // Pino do motor conectado ao PB0
#define BUTTON_PIN PD2 // Pino do botão conectado ao PD2

// Função para inicializar o display em modo barramento
void lcd_init() {
    // Código de inicialização do display
}

void lcd_command(uint8_t cmd) {
    // Função para enviar comandos ao LCD
}

void setup() {
    // Configuração dos pinos
    DDRB |= (1 << MOTOR_PIN);  // Define o pino do motor como saída
    DDRD |= 0xF0;              // Define os pinos PD4 a PD7 como saídas para o LCD
    PORTD |= (1 << BUTTON_PIN); // Habilita pull-up interno para o botão

    lcd_init();  // Inicializa o display
}

void loop() {
    // Verifica se o botão foi pressionado
    if (!(PIND & (1 << BUTTON_PIN))) {
        PORTB |= (1 << MOTOR_PIN); // Liga o motor
        _delay_ms(10000);          // Aguarda 10 segundos
        PORTB &= ~(1 << MOTOR_PIN); // Desliga o motor
    }
}

int main(void) {
    setup();
    while (1) {
        loop();
    }
}