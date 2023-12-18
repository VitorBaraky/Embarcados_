# Trabalho Final Sistemas embarcados 
Grupo = Vitor Baraky e Victor Hugo

## Uso de teclado analógico para configuração de duty cycle
- Materiais utilizados :
    - ESP_32;
    - Teclado Analógico de 12 botões.
      
- Tasks utilizadas :
  - ADC_task (Conversor analógico/digital)
    
    Esta task foi utilizada para realizar a medição no modo "one_shot" de uma entrada analógia proveniente do teclado. O canal utilizado foi o "channel 0" e este valor é enviado via fila para a task      do teclado onde este valor é avaliado.
    
  - pwm_task (Pulse width modulation)

    Esta task foi utilzida para setar o duty cycle do LED vermelho (Channel 2) através dos digitos recebidos da task do teclado(keyboard). O primeiro digito recebido representa as dezenas e o segundo digito representa a unidade, desta forma permitindo a configuração de um duty cycle na faixa de 0%-99%.

  - keyboard_task (Teclado analógico)

    Nesta task foi analisado o valor do ADC para determinar o digito que é enviado para a task do PWM.

  - main_task (Task principal).

    Nesta task foram feitas as configurações e inicializações das tasks e queues utilizadas no trabalho.
    
