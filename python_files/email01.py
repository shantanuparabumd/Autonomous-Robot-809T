import os
from datetime import datetime
import smtplib
from smtplib import SMTP
from smtplib import SMTPException
import email
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage



smtpUser = 'sparab@umd.edu'
smtpPass = 'oeonpzethtnqlxxt'

toAdd = 'ENPM809TS19@gmail.com'
fromAdd = smtpUser
subject = 'Homework 9 Part 2 Submission'
msg = MIMEMultipart()
msg['Subject'] = subject
msg['From'] = fromAdd
msg['To'] = toAdd
msg['Cc'] = 'rpatil10@umd.edu'
msg.preamble = "Homework Submission"

body=MIMEText("This mail servers as a part of homework 9 submission.\n PFA the image recorded after block retrival")
msg.attach(body)

fp=open('hw9.jpg','rb')

img = MIMEImage(fp.read())
fp.close()
msg.attach(img)

s=smtplib.SMTP('smtp.gmail.com',587)
s.ehlo()
s.starttls()
s.ehlo()

s.login(smtpUser,smtpPass)
s.sendmail(fromAdd,toAdd,msg.as_string())
s.quit()
print("Email Delivered")