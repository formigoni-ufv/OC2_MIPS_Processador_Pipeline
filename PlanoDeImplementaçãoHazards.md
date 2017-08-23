Forwarding Unit
===============

Deve detectar se uma instrução ira escrever em registrador que será lido no próximo ciclo. Para isso ele se atenta a flag EscreveReg do registrador MEM/WB e EX/MEM do pipeline.
* Se a flag EscreveReg está ativa para o registrador EX/MEM, compara-se o EX/MEM.registradorRd com 0 e com os registradores de entrada Rs da próxima instrução (ID/EX). Se EX/MEM.registradorRd não for 0:
   * e tem valor similar a ID/EX.registradorRs... Forward! (ForwardA=10)
   * e tem valor similar a ID/EX.registradorRt... Forward! (ForwardB=10)
* Se a flag EscreveReg está ativa para o registrador MEM/WB, compara-se o MEM/WB.registradorRd com 0 e com os registradores de entrada Rs da instrução 2 ciclos seguintes (ID/EX). Se MEM/WB.registradorRd não for 0:
   * confere se não há Forward em MEM antes.¹ Caso não tenha Forward em Mem:
       * e tem valor similar a ID/EX.registradorRs... Forward! (ForwardA=01)
       * e tem valor similar a ID/EX.registradorRt... Forward! (ForwardB=01)
* Caso não haja Forward nenhum então (ForwardB=00)(ForwardA=00)

¹ Conferindo se EX/MEM.EscreveReg está inativo ou se EX/MEM.registradorRd é 0. Indicando que talve haja Forward em Mem. Então confere se o Registrador Rd de escrita em EX/MEM se assemelha com algum Registrador de entrada de ID/EX, Rs ou Rt.


Entradas
--------

* ID/EX.registradorRs
* ID/EX.registradorRt
* EX/MEM.registradorRd
* MEM/WB.registradorRd
* EX/MEM.EscreveReg
* MEM/WB.EscreveReg,

Saidas
------

* ForwardB
* ForwardA

Logica
------

Pagina 297 e 295

Hazard Detection Unit
=====================

Descrição
---------

Deve detectar lw para ocasionar os stalls quando necessário. Para isso ele se atenta a flag LeMem do estágio EX.

* Se a flag LeMem anunciar algum load, então a unidade de detecção de hazard confere os registradores Rs e Rt da instrução do estágio ID, comparando-as com o registrador Rt da instrução de load. Se forem iguais é STALL.

Como funciona o stall? Basicamente ele força no próximo ciclo que todos registradores em ID/EX tenham valor 0 e que os registradore PC e IF/ID tenham o mesmo valor do último ciclo. Atrasando os ciclos

Entradas
--------

* ID/EX.LeMem
* ID/EX.registradorRt
* IF/ID.registradorRs
* IF/ID.registradorRt

Saidas
------

* PCWrite
* IF/ID.Write
* Mux.ID/EX.Write

Logica
------

Pagina 300
