import numpy as np
def changedegree(x):
    string2float=float(x)
    minute=string2float%100
    result=string2float-minute
    result=result/100
    result=result+minute/60
    return result



def radius(x):  #x 
    R=np.sqrt( ((6378000**2*np.cos(x*np.pi/180))**2+( 6356000**2*np.sin(x*np.pi/180))**2)/
               ((6378000*np.cos(x*np.pi/180))**2+( 6356000*np.sin(x*np.pi/180))**2))
    R_2=R*np.cos(x)
    return R, R_2  #R=latitude R_2=longitude
    



aa=changedegree("3652.77")
print(aa)

R1,R2=radius(37.29)

x3=np.cos(37.29)
print(R1)
print(R2)
print(np.pi)
