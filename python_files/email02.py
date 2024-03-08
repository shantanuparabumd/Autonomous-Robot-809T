import imaplib
import email
import time

def cehckEmail():

    mail = imaplib.IMAP4_SSL('imap.gmail.com')
    mail.login('sparab@umd.edu','oeonpzethtnqlxxt')
    mail.list()

    count = 0

    while count <60:
        try:
            mail.select("inbox")

            result, data = mail.search(None, '(UNSEEN FROM "shantanuparab99@gmail.com")')

            print(result)
            print(len(data))


            ids = data[0]
            id_list=ids.split()

            latest_email_id = id_list[-1]

            result, data = mail.fetch(latest_email_id, "(RFC822)")

            if data is None:
                print("Waiting...")

            if data is not None:
                print(data,result,latest_email_id)
                print("Process Inititated!")

        except IndexError:
            time.sleep(2)
            if count < 59:
                count = count +1
                continue
            else:
                print("Gameover")
                count=60
cehckEmail()

