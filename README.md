Projeto para treinamento da estratégia para a equipe de Futebol de Robôs da UTFPR.

Ele está dividido em dois diretórios principais: operation (contém as classes utilizadas para realizar os movimentos da equipe de robôs) e training (na qual estão as classes para treinamento dos agentes).

O padrão para o uso do git é o GitFlow (https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow).

Algumas considerações sobre o git:
- os commits devem ser feitos com o identificador primeiro (número da issue no repositório), dois pontos e a descrição do commit (escrita de forma clara);
- evitar de colocar muito código para as Pull Requests, para facilitar o review;
- testar bem o código antes de submetê-lo a Pull Request.

O padrão de projeto que deve ser seguido é o descrito em https://peps.python.org/pep-0008/#naming-conventions.

Além desses padrões, é importante seguir os seguintes pontos:
- para o padrão de nomenclatura de classes, campos e outros elementos de linguagem, seguir o padrão do C# (https://learn.microsoft.com/en-us/dotnet/csharp/fundamentals/coding-style/identifier-names?source=recommendations);
- utilizar os comentários somente quando necessário;
- colocar comentário "TODO: ..." quando for necessária alguma modificação, melhoramento ou reavaliação de trecho de código;
- evitar o uso de variáveis globais, para facilitar os testes de debugging;
- colocar explicitamente o tipo das variáveis sempre que possível;
- quando uma função tiver 3 ou mais parâmetros, fazer a seguinte quebra de linha:
errado: 
funcao(variavel1: Tipo1, variavel2: Tipo2, variavel3: Tipo3, variavel4: Tipo4)
certo:
funcao(variavel1: Tipo1,
    variavel2: Tipo2,
    variavel3: Tipo3)
- manter as funções com o tamanho menor possível, de prefêrencia, menor que 5 linhas;
- tentar ao máximo dividir os códigos em funções;
- evitar o uso de nomes com siglas ou reduzidas, mesmo que sejam usuais. Por exemplo, em vez de usar config, use configuracao ou configuration;
- levar em conta os princípios de Engenharia de Software, o que inclui Coesão e Acoplamento;
- evitar o hard code. Para strings e valores constantes, procure inseri-los no arquivo configuration.json e carregá-lo no local em que se precisa.