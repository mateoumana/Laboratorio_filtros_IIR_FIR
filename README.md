# Laboratorio_filtros_IIR_FIR
Código en arduino para un filtro pasa banda tipo FIR y filtros resonadores tipo IIR

Para el filtro tipo FIR se implemento con la ayuda de la funcion de Matlab filterDesigner, funcion que proporciona grandes ventajas a la hora de diseñar filtros, principalmente por que ofrece el codigo en c/c++ del vector de coeficientes del filtro. Con dicho filtro se debe prender un LED si las notas musicales de un piano se encentran entre el ancho de banda de 900 y 1100Hz garantizando que en 850 y 1150Hz se tenga almenos una perdida de -8dB.

Para el filtro tipo IIR se usaron resonadores por facilidad, ya que la voz esta compuesta por diferentes componentes frecuenciales, por ende cada resonador se encargara de filtrar cada componente frecuencial de la voz. Con estos filtros se determinara el encendido y apagado de dos LED´S un rojo para cuando se diga NO, y un verde para cuando se diga SI, de lo contrario ambos estarán apagados.

Para el control de encendido y apagado de los LED´S para cada inciso se realiza por medio de la relacion de energias entre la salida de cada filtro y la entrada origianl (Ey/Ex), de tal manera que cuando Ey/Ex > 0.9 (90%) prenda el LED correspondiente.

Todo a una frecuancia de muestreo de 8000Hz.
