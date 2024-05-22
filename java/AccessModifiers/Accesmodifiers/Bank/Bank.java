//modificadores de acceso --Getter(obtener)/ Seter (asignar)

public class Bank{
    private CheckingAccount accountOne; // Check... Clase y account.. atributos
    private CheckingAccount accountTwo;

    public Bank(){
        accountOne = new CheckingAccount("Zeus", 1000, "1");
        accountTwo = new CheckingAccount("Hades", 200, "2");

    }

    public static void main(String [] args){
        Bank bankOfGods = new Bank();  //objeto en este caso 
        System.out.println("Saldo actual:" + " " + bankOfGods.accountOne.getBalance() ); //imprime 100 por el balance establecido arriba
        // bankOfGods.accountOne.setBalance(5000); //remplaza el valor establecido antes. (Como si se borrara el 1000 de inputBalance)
        // System.out.println(bankOfGods.accountOne.getBalance());
        // System.out.println(bankOfGods.accountOne.getMonthlyInterest()); //interes
        
       
        bankOfGods.accountOne.consignar(1000); 
        System.out.println("Su saldo actual es de: "+ bankOfGods.accountOne.getBalance());
        bankOfGods.accountOne.consignar(50); 
        System.out.println("Su saldo actual es de: "+ bankOfGods.accountOne.getBalance());
        bankOfGods.accountOne.consignar(100);
        System.out.println("Su saldo actual es de: "+ bankOfGods.accountOne.getBalance());
        
        bankOfGods.accountOne.retirar(100);
        System.out.println("Su saldo actual es de: " + bankOfGods.accountOne.getBalance());
        bankOfGods.accountOne.retirar(50);
        System.out.println("Su saldo actual es de: " + bankOfGods.accountOne.getBalance());
        bankOfGods.accountOne.retirar(100);
        System.out.println("Su saldo actual es de: " + bankOfGods.accountOne.getBalance());

        }
}
