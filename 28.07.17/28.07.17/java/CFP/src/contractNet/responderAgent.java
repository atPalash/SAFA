package contractNet;

import jade.core.Agent;
import jade.core.behaviours.*;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.ServiceDescription;

import java.io.*;
import java.nio.file.*;
import java.util.*;

public class responderAgent extends Agent {
    // The catalogue of books for sale (maps the title of a book to its price)
    private String capability;
    private int efficiency;
    // The GUI by means of which the user can add books in the catalogue
    private responderGui myGui;

    // Put agent initializations here
    protected void setup() {
        // Create and show the GUI
        myGui = new responderGui(this);
        myGui.showGui();

        // Register the book-selling service in the yellow pages
        DFAgentDescription dfd = new DFAgentDescription();
        dfd.setName(getAID());
        ServiceDescription sd = new ServiceDescription();
        sd.setType("job");
        sd.setName("JADE-job-trading");
        dfd.addServices(sd);
        try {
            DFService.register(this, dfd);
        }
        catch (FIPAException fe) {
            fe.printStackTrace();
        }

        // Add the behaviour serving queries from buyer agents
        addBehaviour(new OfferRequestsServer());

        // Add the behaviour serving purchase orders from buyer agents
        addBehaviour(new PurchaseOrdersServer());
    }

    // Put agent clean-up operations here
    protected void takeDown() {
        // Deregister from the yellow pages
        try {
            DFService.deregister(this);
        }
        catch (FIPAException fe) {
            fe.printStackTrace();
        }
        // Close the GUI
        myGui.dispose();
        // Printout a dismissal message
        System.out.println("responder-agent "+getAID().getName()+" terminating.");
    }

    /**
     This is invoked by the GUI when the user adds a new book for sale
     */
    public void updateCapability(final String title, final int efficacy) {
        addBehaviour(new OneShotBehaviour() {
            public void action() {
                capability = title;
                efficiency = efficacy;
                System.out.println(title+" inserted into catalogue. efficiency = "+efficacy);
            }
        } );
    }

    /**
     Inner class OfferRequestsServer.
     This is the behaviour used by Book-seller agents to serve incoming requests
     for offer from buyer agents.
     If the requested book is in the local catalogue the seller agent replies
     with a PROPOSE message specifying the price. Otherwise a REFUSE message is
     sent back.
     */
    private class OfferRequestsServer extends CyclicBehaviour {
        public void action() {
            MessageTemplate mt = MessageTemplate.MatchPerformative(ACLMessage.CFP);
            ACLMessage msg = myAgent.receive(mt);
            if (msg != null) {
                // CFP Message received. Process it
                String title = msg.getContent();
                ACLMessage reply = msg.createReply();

                if (Objects.equals(title, capability)) {
                    // The requested book is available for sale. Reply with the price
                    reply.setPerformative(ACLMessage.PROPOSE);
                    reply.setContent(String.valueOf(efficiency));
                }
                else {
                    // The requested book is NOT available for sale.
                    reply.setPerformative(ACLMessage.REFUSE);
                    reply.setContent("not-available");
                }
                myAgent.send(reply);
            }
            else {
                block();
            }
        }
    }  // End of inner class OfferRequestsServer

    /**
     Inner class PurchaseOrdersServer.
     This is the behaviour used by Book-seller agents to serve incoming
     offer acceptances (i.e. purchase orders) from buyer agents.
     The seller agent removes the purchased book from its catalogue
     and replies with an INFORM message to notify the buyer that the
     purchase has been sucesfully completed.
     */
    private class PurchaseOrdersServer extends CyclicBehaviour {
        public void action() {
            MessageTemplate mt = MessageTemplate.MatchPerformative(ACLMessage.ACCEPT_PROPOSAL);
            ACLMessage msg = myAgent.receive(mt);
            if (msg != null) {
                // ACCEPT_PROPOSAL Message received. Process it
                String title = msg.getContent();
                ACLMessage reply = msg.createReply();

                if (Objects.equals(title, capability)) {
                    reply.setPerformative(ACLMessage.INFORM);
                    System.out.println(title+" sold to agent "+msg.getSender().getName());
                    String sentence;
                    try {
                        String fs = System.getProperty("file.separator");
                        //String path = /home/pi/Desktop/testingFiles/27.07.17_mobile/MyFile.txt
                        String path_javaReply = fs + "home" + fs + "pi" + fs + "Desktop" + fs + "testingFiles" + fs + "28.07.17_mobile" + fs + "java_reply.txt";
                        FileWriter writer = new FileWriter(path_javaReply, false);
                        sentence = title;
                        BufferedWriter bufferedWriter = new BufferedWriter(writer);
                        bufferedWriter.write(sentence);
                        bufferedWriter.close();
                        WatchService watcher = FileSystems.getDefault().newWatchService();
                        Path dir = Paths.get(fs + "home" + fs + "pi" + fs + "Desktop" + fs + "testingFiles" + fs + "28.07.17_mobile");
                        dir.register(watcher, StandardWatchEventKinds.ENTRY_MODIFY);
                        int count = 0;
                        while (true){
                            count++;
                            System.out.println(count);
                            WatchKey key;
                            try {
                                key = watcher.take();
                            } catch (InterruptedException ex) {
                                return;
                            }
                            for (WatchEvent<?> event : key.pollEvents()) {
                                WatchEvent.Kind<?> kind = event.kind();
                                @SuppressWarnings("unchecked")
                                WatchEvent<Path> ev = (WatchEvent<Path>) event;
                                Path fileName = ev.context();
                                System.out.println(fileName);
                                //if(count%2==0){
                                if (kind == StandardWatchEventKinds.OVERFLOW) {
                                    System.out.println("overflow error");
                                }
                                else if (kind == StandardWatchEventKinds.ENTRY_MODIFY && Objects.equals(fileName.toString(), "python_reply.txt")) {
                                    System.out.println("File modified" + fileName);
                                    String path_r = fs + "home" + fs + "pi" + fs + "Desktop" + fs + "testingFiles" + fs + "28.07.17_mobile" + fs +  "python_reply.txt";
                                    BufferedReader fr = new BufferedReader(new FileReader(path_r));
                                    String line = fr.readLine();
                                    System.out.println(line);
                                    if(Objects.equals(line, "reached start")){
                                        reply.setContent(line);
                                        myAgent.send(reply);
                                    }
                                    else if(Objects.equals(line, "loaded start")){
                                        reply.setContent(line);
                                        myAgent.send(reply);
                                    }
                                }
                                //}
                            }
                            boolean valid = key.reset();
                            if (!valid) {
                                break;
                            }
                        }
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
                else {
                    // The requested book has been sold to another buyer in the meanwhile .
                    reply.setPerformative(ACLMessage.FAILURE);
                    reply.setContent("not-available");
                    myAgent.send(reply);
                }
            }
            else {
                block();
            }
        }
    }  // End of inner class OfferRequestsServer
}
