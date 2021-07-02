import operator
filenames = ['DD','DI', 'TD', 'TI']
for name in filenames:
    file = open(name+'.txt')
    numeros = {}
    for line in file:
        if (numeros.get(line.strip())==None):
            numeros[line.strip()]=1
        else:
            numeros[line.strip()] = numeros.get(line.strip()) +1
    numeros_sort = sorted(numeros.items(), key=operator.itemgetter(1), reverse=True)
    print('Valores mas repetidos: ',numeros_sort[:5])

    cantidad =0
    suma = 0
    for n in numeros_sort[:5]:
        suma = int(n[0])*int(n[1]) + suma
        cantidad = cantidad + int(n[1])

    print('Promedio : ', round(suma/cantidad))

    relacion = (60000000/(suma/cantidad))/330
    print('Relacion de engranajes para %s: ' %name, round(relacion))
    file.close()