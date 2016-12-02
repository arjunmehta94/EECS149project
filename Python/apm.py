print 'Start Script'
for chan in range(1,9):
    Script.SendRC(chan,1500,False)
    Script.SendRC(3,Script.GetParam('RC3_MIN'),True)
    Script.Sleep(5000)

while cs.lat == 0:
    print 'Waiting for GPS'
    Script.Sleep(1000)
    print 'Got GPS'
    jo = 10 * 13
    print jo
    Script.SendRC(3,1000,False)
    Script.SendRC(4,2000,True)
    cs.messages.Clear()
    Script.WaitFor('ARMING MOTORS',30000)
    Script.SendRC(4,1500,True)
    print 'Motors Armed!'
    Script.SendRC(3,1700,True)
