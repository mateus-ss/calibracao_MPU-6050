# Calibração Estática do Acelerômetro MPU-6050 e Medição de Ângulo de Inclinação em Plataforma de Validação

Este projeto descreve a calibração estática e a validação experimental de um inclinômetro embarcado baseado no acelerômetro triaxial MEMS **MPU-6050 (GY-521)** e no microcontrolador **ESP32 DevKit**.

A caracterização nas seis orientações ortogonais (±1 g por eixo) revelou um offset elevado no eixo X (+1,322 g), motivando a adoção da escala ±4 g (8192 LSB/g) com filtro passa-baixas digital (DLPF) de 5 Hz. Os ganhos estimados (0,98591; 0,98939; 1,01413) apresentaram desvio inferior a 1,5% do valor unitário.

O ângulo foi estimado via função `atan2` no plano XZ e validado em plataforma impressa em 3D com transferidor entre 10° e 140°, atingindo alta precisão eletrônica (σ médio de 0,129°; R² = 0,9989), RMSE de 7,228° e erro máximo de 10,862°, dominados por efeitos mecânicos do aparato.

**Palavras-chave:** Acelerômetro MEMS, calibração estática, ESP32, instrumentação eletrônica, medição de inclinação, MPU-6050.
