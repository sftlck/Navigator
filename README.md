# Navigator


<img src="navigator-v0.bmp" alt="Exemplo imagem">

> Uma versão digital do comportamento da Zeiss Prismo Navigator com uma pitada de álgebra linear.

### Ajustes e melhorias

Neste ponto do projeto:

- [x] Modelos .STL da máquina
- [x] Movimentos de translação por inputs do teclado
- [x] Movimentos de translação por CNC
- [x] Criação de objetos geométricos no espaço
- [ ] Criação de sub-sistemas de coordenadas

## 💻 Pré-requisitos

Antes de começar, verifique se você atendeu aos seguintes requisitos:

- Você instalou a versão mais recente da biblioteca `<vtk / Python 3.11 ou acima>`

## 🚀 Instalando Navigator

Para instalar o Navigator, siga estas etapas:

Windows:

- Baixe os 4 arquivos em formato .STL
- Baixe o arquivo Navigator.py e deixe-o no mesmo diretório dos arquivos .STL
- Execute o código!

## ☕ Usando Navigator

Para usar o Navigator, siga estas etapas:

- Use as setas para se movimentar no espaço
- Use "1" para registrar as coordenadas de um ponto no espaço
- Use "2" para criar uma Linha usando os dois últimos pontos registrados
- Use "4" para criar um círculo e fazer a máquina executar a trajetória de suas bordas usando os três últimos pontos registrados
- Use "6" para calcular o ângulo interno e externo entre dois vetores usando os últimos 4 pontos registrados
- Use "7" para criar um plano usando os três últimos pontos registrados
