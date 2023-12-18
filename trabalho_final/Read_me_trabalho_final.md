# Trabalho Final Sistemas embarcados - Grupo = Vitor Baraky e Victor Hugo

## Uso de teclado analógico para configuração de duty cycle
- Materiais utilizados :
    - ESP_32;
    - Teclado Analógico de 12 botões.
    - 
- Tasks utilizadas :
  - ADC_task (Conversor analógico/digital)
    
    Esta task foi utilizada para realizar a medição no modo "one_shot" de uma entrada analógia proveniente do teclado. O canal utilizado foi o "channel 0" e este valor é enviado via fila para a task      do teclado.
    
  - pwm_task (Pulse width modulation);
  - main_task (Task principal).
  
    
