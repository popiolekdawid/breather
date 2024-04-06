from ftplib import parse150
import time
from tkinter import ttk 
import tkinter as Tk
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# from Backend import Backend  #self written data acquisition handler  
#import random
import spidev

spi_bus = 0
spi_device = 0

spi = spidev.SpiDev()
spi.open(spi_bus, spi_device)
spi.max_speed_hz = 500000



#global variables
t0 = datetime.now() #start time of program
data = np.array([])
t = []
obj = []
cis = []
tl = []
prze = []
send_data=[]
a=[0,0,0,0,0,0,0,0,0,0]
loc=0
tryb={'Objetosc [ml]':1,'Cisnienie [cmH2O]':2 }
status = {0 : 'BLAD Polaczenia',1 : 'Polaczono', 2 : 'Wyslano', 3 : 'BLAD URZADZENIA - przerwany obwod', 4 : 'BLAD URZADZENIA - zatkany obwod'}
tr=0
twd=0
twy=0
peep=0
p1=45.5


#Returns time difference in seconds
def time_difference(t1, t2):
    delta = t2-t1
    delta = delta.total_seconds()
    return delta

def status_polaczenia():
    for i in range(2):
        send_data[0:]=[10]
        send_data[1:]=[0]
        send_data[2:]=[0]
        send_data[3:]=[0]
        send_data[4:]=[0]
        send_data[5:]=[0]
        send_data[6:]=[0]
        send_data[7:]=[0]
        send_data[8:]=[0]
        send_data[9:]=[20]
        
        a=spi.xfer(send_data)
    if (a[7]==30):
        return 1
    else:
        return 0
def stop():
    global tr
    tr=0
def send():
    global loc, tryb, tr, twd, twy, peep, etykieta_wys, a, l7
    loc=int(scale.get())#&0xFFFF
    tr=int(tryb[rb_var.get()])
    twd=int(((60/scale2.get())/(1+scale3.get()))*1000)
    twy=int(scale3.get()*1000)
    peep=int(scale4.get())
    #etykieta_wys.set('Wysylam')
    #time.sleep(1)
    l7.set(scale.get())
    l9.set(scale2.get())
    l11.set(scale3.get())
    l13.set(scale4.get())
    l6.set(rb_var.get())
    for i in range(2):
        send_data[0:]=[1]
        send_data[1:]=[tr]
        send_data[2:]=[loc>>8]
        send_data[3:]=[loc&0xFF]
        send_data[4:]=[twd>>8]
        send_data[5:]=[twd&0xFF]
        send_data[6:]=[twy>>8]
        send_data[7:]=[twy&0xFF]
        send_data[8:]=[peep]
        send_data[9:]=[2]
        
        a=spi.xfer(send_data)
        
        
    if (a[7]==3):
        etykieta_wys.set(status[2])
    else:
        etykieta_wys.set(status[0])
    
   
    
def upload():
    global loc, tr, twd, twy, peep
    objetosc = 0
    cisnienie = 0
    tlen = 0
    przeplyw = 0
    send_data[0:]=[1]
    send_data[1:]=[tr]
    send_data[2:]=[loc>>8]
    send_data[3:]=[loc&0xFF]
    send_data[4:]=[twd>>8]
    send_data[5:]=[twd&0xFF]
    send_data[6:]=[twy>>8]
    send_data[7:]=[twy&0xFF]
    send_data[8:]=[peep]
    send_data[9:]=[2]
    #spi.writebytes(send_data)
    #a = spi.readbytes(6)
    a=spi.xfer(send_data)
    if (a[8]==1):
        etykieta_wys.set(status[3])
    if (a[8]==2):
        etykieta_wys.set(status[4])
    if (a[0] == 1 and a[9] == 2):
        objetosc = a[1]<<8 | a[2]
        cisnienie = a[3]
        przeplyw = ((a[5]<<8 | a[6])*60)/1000
    #k_przesylu = a[7]
    
    #objetosc =float(scale.get())
    #cisnienie = random.uniform(1, 30)
    #objetosc = random.uniform(1, 2000)
    #przeplyw = random.uniform(1, 2000)

    obj.append(objetosc)
    cis.append(cisnienie)
    #tl.append(tlen)
    prze.append(przeplyw)
#    print(send_data)


def scale_changed(ch):
    objetos_u.set(ch)

def scale2_changed(c):
    if scale2.get()==0:
        czas_wdechu.set(0)
        czas_wydechu.set(0)
    else:
        czas_wdechu.set("%.3f" % (float(60/scale2.get())/(1+scale3.get())))
        czas_wydechu.set("%.3f" % (float(60/scale2.get())/(1+scale3.get())*scale3.get()))

    
def cis_zakres():
    global scale
    scale.destroy()
    scale = Tk.Scale(tab1, from_ = 0, to = 40, orient=Tk.HORIZONTAL, showvalue=0, width = 30)#, label=etykieta.get())
    scale.place(x = 420,y=30, width = 350)
    objetos_u.set(0)
    scale.config(command = scale_changed)
    
def obj_zakres():
    global scale
    scale.destroy()
    scale = Tk.Scale(tab1, from_ = 0, to = 2000, orient=Tk.HORIZONTAL, showvalue=0, width = 30)#, label=etykieta.get())
    scale.place(x = 420,y=30, width = 350)
    objetos_u.set(0)
    scale.config(command = scale_changed)

def update():
    
    t.append(time_difference(t0, datetime.now()))
#     a = spi.readbytes(6)
#     objetosc = a[0]<<8 | a[1]
#     cisnienie = a[2]
#     #tlen = a[3]
#     przeplyw = a[4]<<8 | a[5]
     
    
    #objetosc = loc
    #cisnienie = random.uniform(1, 5)
    ##tlen = random.uniform(1, 5)
    #przeplyw = random.uniform(1, 5)
 
    #obj.append(objetosc)
    #cis.append(cisnienie)
    ##tl.append(tlen)
    #prze.append(przeplyw)
    upload()
    if len(t) > 200:
        del t[0]
        del obj[0]
        del cis[0]
        #del tl[0]
        del prze[0]
   
    line1[0].set_data(np.arange(0, len(obj)), obj)
    canvas1.restore_region(background)
    ax1.draw_artist(line1[0])
    canvas1.blit(ax1.bbox)
    canvas1.flush_events()

    line2[0].set_data(np.arange(0, len(cis)), cis)
    canvas2.restore_region(background)
    ax2.draw_artist(line2[0])
    canvas2.blit(ax2.bbox)
    canvas2.flush_events()

    #line3[0].set_data(np.arange(0, len(tl)), tl)
    #canvas3.restore_region(background)
    #ax3.draw_artist(line3[0])
    #canvas3.blit(ax3.bbox)
    #canvas3.flush_events()
    
    line4[0].set_data(np.arange(0, len(prze)), prze)
    canvas4.restore_region(background)
    ax4.draw_artist(line4[0])
    canvas4.blit(ax4.bbox)
    canvas4.flush_events()

    tab2.after(1,update)

def przelicz_k():
    global p1
    p1=45.5


def przelicz_m():
    global p1
    p1=50



def przelicz():
    global p1
    PBW = int ((p1 + 0.91*(int(scale33.get()) - 152.4))*8)
    PBV_var.set(PBW)
    #p1 = Tk.StringVar() 
    #p2 = Tk.StringVar() 
    #p1.set("Kobieta")
    #p2.set("Mezczyzna")
    #if rb_plec==p1:
    #    PBW = 45.5 + 0.91*(int(scale33.get()) - 152.4)
    #    PBV_var.set(PBW)
    #else:
    #    PBW = 50 + 0.91*(int(scale33.get()) - 152.4)
    #    PBV_var.set(PBW)

def close() :
            print ("close")
            root.quit()
            root.destroy()
            



# Mainloop
root= Tk.Tk()
width= root.winfo_screenwidth()               
height= root.winfo_screenheight()               
#root.geometry('%dx%d' % (width, height))
root.attributes('-fullscreen',True)

style = ttk.Style()
style.theme_create( "MyStyle", parent="alt", settings={
        "TNotebook": {"configure": {"tabmargins": [2, 5, 2, 0] } },
        "TNotebook.Tab": {"configure": {"padding": [10, 10] },}})

style.theme_use("MyStyle")

tab_control = ttk.Notebook(root, width=300, height = 300) 
tab1 = ttk.Frame(tab_control) 
tab2 = ttk.Frame(tab_control) 
tab3 = ttk.Frame(tab_control) 
tab_control.add(tab3, text='Ustawienia Wstepne') 
tab_control.add(tab1, text='Ustawienia') 
tab_control.add(tab2, text='Wykresy') 
tab_control.pack(expand=1, fill='both') 

# menubar = Tk.Menu(root)
# filemenu = Tk.Menu(menubar, tearoff=0)
# filemenu.add_command(label="Exit", command=close)
# menubar.add_cascade(label="File", menu=filemenu)
# root.config(menu=menubar)

scale32 = Tk.Scale(tab3, from_ = 0, to = 100, resolution=1, orient=Tk.HORIZONTAL, showvalue=1, label='Wiek', width = 30)
scale32.place(x = 10,y=80, width = 400)

scale33 = Tk.Scale(tab3, from_ = 0, to = 200, resolution=1, orient=Tk.HORIZONTAL, showvalue=1, label='Wzrost [cm]', width = 30)
scale33.place(x = 10,y=180, width = 400)

PBV_var = Tk.StringVar() 
label = Tk.Label(tab3, textvariable = PBV_var) 
label.place(x=400,y=400)

b2 = Tk.Button(tab3, text="Przelicz", width=30, command=przelicz, bg='lightgreen')
b2.place(x = 570,y=70, width = 70, height = 50)

#etykieta = Tk.StringVar()
scale = Tk.Scale(tab1, from_ = 0, to = 2000, orient=Tk.HORIZONTAL, showvalue=0, width = 30)#, label=etykieta.get())
scale.place(x = 420,y=30, width = 350)
objetos_u = Tk.StringVar()
label = Tk.Label(tab1, textvariable = objetos_u) 
label.place(x=700,y=10)
objetos_u.set(0)
scale.config(command = scale_changed)

scale2 = Tk.Scale(tab1, from_ = 0, to = 40, resolution=1, orient=Tk.HORIZONTAL, showvalue=1, label='Czestosc [oddechy/min]', width = 30)
scale2.place(x = 10,y=80, width = 400)
czas_wdechu = Tk.StringVar()
l14 = Tk.StringVar()
l14.set('Czas wdechu [s]')
label14= Tk.Label(tab1, textvariable = l14) 
label14.place(x=500,y=210)
label15 = Tk.Label(tab1, textvariable = czas_wdechu) 
czas_wdechu.set(0)
label15.place(x=620,y=210)
czas_wydechu = Tk.StringVar()
l16 = Tk.StringVar()
l16.set('Czas wydechu [s]')
label16= Tk.Label(tab1, textvariable = l16) 
label16.place(x=500,y=240)
label17 = Tk.Label(tab1, textvariable = czas_wydechu) 
czas_wydechu.set(0)
label17.place(x=620,y=240)
scale2.config(command = scale2_changed)

scale3 = Tk.Scale(tab1, from_ = 0, to = 5, resolution=0.1, orient=Tk.HORIZONTAL, showvalue=1, label='I:E [-]', width = 30)
scale3.place(x = 10,y=180, width = 400)
scale3.config(command = scale2_changed)

scale4 = Tk.Scale(tab1, from_ = 0, to = 20, orient=Tk.HORIZONTAL, showvalue=1, label='PEEP [cmH2O]', width = 30)
scale4.place(x = 10,y=280, width = 400)

b1 = Tk.Button(tab1, text="Wyslij", width=30, command=send, bg='lightgreen')
b1.place(x = 570,y=70, width = 70, height = 50)

etykieta_wys=Tk.StringVar()
#etykieta_wys.set(status[status_polaczenia()])

b2 = Tk.Button(tab1, text="Zamknij", width=30, command=close, bg='red')
b2.place(x = 670,y=370, width = 70, height = 50)

b3 = Tk.Button(tab1, text="STOP", width=30, command=stop, bg='red')
b3.place(x = 670,y=270, width = 70, height = 50)


#Tab1#
rb_var = Tk.StringVar() 
rb_obj = Tk.Radiobutton(tab1, variable = rb_var, value = "Objetosc [ml]", text = "Objetosc", command=obj_zakres, height= 2)
#rb_prz = Tk.Radiobutton(tab1, variable = rb_var, value = "Przeplyw", text = "Przeplyw",  height= 2)#, command=tryb_o)
rb_cis = Tk.Radiobutton(tab1, variable = rb_var, value = "Cisnienie [cmH2O]", text = "Cisnienie", command=cis_zakres,  height= 2)
rb_var.set("Objetosc [ml]") 
rb_obj.place(x = 20,y=20)
#rb_prz.place(x = 550,y=240)
rb_cis.place(x = 140,y=20)
#rb_var.trace_add("write")
label = Tk.Label(tab1, textvariable = rb_var) 
label.place(x=420,y=10)

#Tab2#
#wykres 1
fig1 = plt.figure(figsize=(4,2))
ax1 = fig1.add_subplot(111)
ax1.set_title('Objetosc [ml]')
ax1.get_xaxis().set_visible(False)
#ax1.set_ylabel('Objetosc')
ax1.axis([0,200,0,1000])
line1 = ax1.plot(t, obj, '-')
canvas1 = FigureCanvasTkAgg(fig1, master = tab2)
canvas1.get_tk_widget().place(x = 10,y=10, width = 400,height = 200)
canvas1.draw()
ax1.add_line(line1[0])
background = canvas1.copy_from_bbox(ax1.bbox)
        
#wykres 2
fig2 = plt.figure(figsize=(4,2))
ax2 = fig2.add_subplot(111)
ax2.set_title('Cisnienie [cmH2O]')
ax2.get_xaxis().set_visible(False)
#ax2.set_ylabel('Cisnienie')
ax2.axis([0,200,0,50])
line2 = ax2.plot(t, cis, '-')
canvas2 = FigureCanvasTkAgg(fig2, master = tab2)
canvas2.get_tk_widget().place(x = 10,y=210, width = 400,height = 200)
canvas2.draw()
ax2.add_line(line2[0])
background = canvas2.copy_from_bbox(ax2.bbox)

##wykres 3
#fig3 = plt.figure(figsize=(4,2))
#ax3 = fig3.add_subplot(111)
#ax3.set_title('Tlen')
#ax3.get_xaxis().set_visible(False)
#ax3.set_ylabel('Tlen')
#ax3.axis([0,200,0,100])
#line3 = ax3.plot(t, tl, '-')
#canvas3 = FigureCanvasTkAgg(fig3, master = tab2)
#canvas3.get_tk_widget().place(x = 410,y=10, width = 400,height = 200)
#canvas3.draw()
#ax3.add_line(line3[0])
#background = canvas3.copy_from_bbox(ax3.bbox)

#wykres 4
fig4 = plt.figure(figsize=(4,2))
ax4 = fig4.add_subplot(111)
ax4.set_title('Przeplyw [l/min]')
ax4.get_xaxis().set_visible(False)
#ax4.set_ylabel('Przeplyw')
ax4.axis([0,200,0,70])
line4 = ax4.plot(t, prze, '-')
canvas4 = FigureCanvasTkAgg(fig4, master = tab2)
canvas4.get_tk_widget().place(x = 410,y=210, width = 400,height = 200)
canvas4.draw()
ax4.add_line(line4[0])
background = canvas4.copy_from_bbox(ax4.bbox)

l51 = Tk.StringVar()
l51.set('Status:')
label8 = Tk.Label(tab2, textvariable = l51) 
label8.place(x=500,y=0)

label5 = Tk.Label(tab2, textvariable = etykieta_wys) 
label5.place(x=540,y=00)

l6 = Tk.StringVar()
label6 = Tk.Label(tab2, textvariable = l6) 
label6.place(x=450,y=50)

l7 = Tk.StringVar()
label7 = Tk.Label(tab2, textvariable = l7) 
label7.place(x=450,y=70)

l8 = Tk.StringVar()
l8.set('Czestosc [oddechy/min]')
label8 = Tk.Label(tab2, textvariable = l8) 
label8.place(x=450,y=100)

l9 = Tk.StringVar()
label9 = Tk.Label(tab2, textvariable = l9) 
label9.place(x=450,y=120)

l10 = Tk.StringVar()
l10.set('I:E [-]')
label10 = Tk.Label(tab2, textvariable = l10) 
label10.place(x=600,y=50)

l11 = Tk.StringVar()
label11 = Tk.Label(tab2, textvariable = l11) 
label11.place(x=600,y=70)

l12 = Tk.StringVar()
l12.set('PEEP [cmH2O]')
label12= Tk.Label(tab2, textvariable = l12) 
label12.place(x=600,y=100)

l13 = Tk.StringVar()
label13 = Tk.Label(tab2, textvariable = l13) 
label13.place(x=600,y=120)

tab2.after(1, update)
root.protocol( "WM_DELETE_WINDOW", close )
root.mainloop()