# Navigator

Este é um projeto da aproximação do comportamento geométrico, desconsiderando fatores de influência externas, da Máquina de Medição por Coordenadas Zeiss Prismo Navigator. Aqui implementei consequências do aprendizado que tive realizando medições em um Laboratório de Metrologia com essa belezinha :v:

Projeto oriundo de minha curiosidade em entender essas máquinas, atrelada ao tempo livre entre 00:00 e 06:00 :heart_eyes:

<img src="navigator-v0.bmp" alt="Exemplo imagem">

> Uma versão digital da Máquina de Medição por Coordenadas Zeiss Prismo Navigator com uma pitadinha de computação gráfica :heart:

Neste ponto do projeto:

- [x] Modelos .STL da máquina
- [x] Movimentos de translação por inputs do teclado
- [x] Movimentos de translação por CNC
- [x] Criação de objetos geométricos no espaço
- [x] Reproduzir o código de cores do Zeiss Calypso :kissing_heart:
- [ ] Criação de sub-sistemas de coordenadas
- [ ] Implementar ajuste por mínimos quadrados para Linhas
- [ ] Implementar ajuste por mínimos quadrados para Planos

## :eyes: Veja no YouTube! :eyes:

[![Navigator Versão 0](https://i3.ytimg.com/vi/Ub5H_4DRlw8/maxresdefault.jpg)](https://www.youtube.com/watch?v=Ub5H_4DRlw8 "Navigator Versão 0")

> Se inscreve no canal pois eu posto atualizações do projeto por lá :yum::yum:

## 💻 Pré-requisitos

Antes de começar, verifique se você atendeu aos seguintes requisitos:

- Você instalou a versão mais recente da biblioteca `<vtk / Python 3.11 ou acima>`
- Você possui um computador com uma placa de vídeo com ao menos 2Gb de VRAM

Atenção! O projeto foi desenvolvido em um computador com as seguintes especificações:
- Processador Ryzen 5 8500G 6 x 12 3551 MHz
- 8 Gb RAM DDR5
- MoBo MSI A620M-E
- Win 11 Pro v10.0.22631 Comp 22631

## 🚀 Instalando Navigator

Para instalar o Navigator, siga estas etapas:

- Baixe os 4 arquivos em formato .STL
- Baixe o arquivo Navigator.py e deixe-o no mesmo diretório dos arquivos .STL
- Execute o código!

## ☕ Usando Navigator

Para usar o Navigator, siga estas etapas:

- Use as setas para se movimentar no espaço
- Use "1" para registrar as coordenadas de um Ponto no espaço
- Use "2" para criar uma Linha usando os dois últimos pontos registrados
- Use "4" para criar um Círculo e fazer a máquina executar a trajetória de suas bordas usando os três últimos pontos registrados
- Use "6" para calcular o ângulo interno e externo entre dois vetores usando os últimos 4 pontos registrados
- Use "7" para criar um Plano usando os três últimos pontos registrados
